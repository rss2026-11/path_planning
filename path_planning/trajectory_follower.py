import rclpy
import numpy as np
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from .utils import LineTrajectory


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        # FILL IN #
        self.lookahead = 1.0        # Lookahead distance in meters
        self.speed = 2.0            # Driving speed in m/s
        self.wheelbase_length = 0.33 # Typical 1/10th scale racecar wheelbase in meters

        self.initialized_traj = False
        self.trajectory = LineTrajectory(self, "/followed_trajectory")

        self.pose_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        self.at_end = False

    def euler_from_quaternion(self, x, y, z, w):
        """ Convert quaternion to euler yaw angle """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def pose_callback(self, odometry_msg):
        if not self.initialized_traj or len(self.trajectory.points) < 2 or self.at_end:
            return

        # 1. Extract Car Position and Yaw
        car_x = odometry_msg.pose.pose.position.x
        car_y = odometry_msg.pose.pose.position.y
        qx = odometry_msg.pose.pose.orientation.x
        qy = odometry_msg.pose.pose.orientation.y
        qz = odometry_msg.pose.pose.orientation.z
        qw = odometry_msg.pose.pose.orientation.w
        
        yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        car_pos = np.array([car_x, car_y])
        # self.get_logger().info(f"car_pos: {car_pos}")

        # Safely extract path points as a 2D numpy array
        try:
            path_pts = np.array([[p.x, p.y] if hasattr(p, 'x') else [p[0], p[1]] for p in self.trajectory.points])
        except AttributeError:
            self.get_logger().error("Could not parse trajectory points structure.")
            return

        # Vectorized nearest segment search
        segments_start = path_pts[:-1]
        segments_end = path_pts[1:]
        
        V = segments_end - segments_start
        W = car_pos - segments_start
        
        l2 = np.sum(V**2, axis=1)
        l2 = np.where(l2 == 0, 1e-6, l2)  # Avoid division by zero
        
        # Calculate parameterized t and clamp between 0 and 1
        t_proj = np.sum(W * V, axis=1) / l2
        t_proj = np.clip(t_proj, 0.0, 1.0)
        
        # Find closest point on each segment and the distance to the car
        projections = segments_start + t_proj[:, np.newaxis] * V
        distances = np.linalg.norm(car_pos - projections, axis=1)
        
        # Find index of the absolute closest segment
        closest_idx = np.argmin(distances)

        # Find the Lookahead Point (Intersection of circle and line segment)
        lookahead_point = None
        r = self.lookahead
        Q = car_pos

        # Start searching forward from the closest segment
        for i in range(closest_idx, len(path_pts) - 1):
            P1 = path_pts[i]
            P2 = path_pts[i+1]
            V_seg = P2 - P1

            a = np.dot(V_seg, V_seg)
            b = 2 * np.dot(V_seg, P1 - Q)
            c = np.dot(P1, P1) + np.dot(Q, Q) - 2 * np.dot(P1, Q) - r**2

            disc = b**2 - 4 * a * c

            # If discriminant is >= 0, there is an intersection
            if disc >= 0:
                sqrt_disc = math.sqrt(disc)
                t1 = (-b + sqrt_disc) / (2 * a)
                t2 = (-b - sqrt_disc) / (2 * a)

                valid_t = []
                if 0 <= t1 <= 1:
                    valid_t.append(t1)
                if 0 <= t2 <= 1:
                    valid_t.append(t2)

                if valid_t:
                    # Choose the larger t so we pick the point further ahead on the segment
                    t_intersect = max(valid_t)
                    lookahead_point = P1 + t_intersect * V_seg
                    break 

        # In case we're at the end of the path and the lookahead circle misses the end
        if lookahead_point is None:
            # lookahead_point = path_pts[-1]
            self.at_end = True
            self.get_logger().info(f"Found end point... stopping.")


        # Transform Lookahead Point to Car's Local Frame
        dx = lookahead_point[0] - car_x
        dy = lookahead_point[1] - car_y

        # Rotate by -yaw to convert to local frame
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        # Calculate Steering Angle using Pure Pursuit Equation
        # Curvature formula: 2 * y_local / (L_d ^ 2)
        curvature = (2.0 * local_y) / (self.lookahead ** 2)
        steering_angle = math.atan(curvature * self.wheelbase_length)

        # self.get_logger().info(f"Steering angle: {steering_angle}")

        # Publish Drive Command
        drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = self.get_clock().now().to_msg()
        # drive_msg.header.frame_id = "base_link"

        if self.at_end:
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
        else:
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = float(self.speed)

        self.drive_pub.publish(drive_msg)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()