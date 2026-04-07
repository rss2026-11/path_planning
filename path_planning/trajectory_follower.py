import rclpy
import numpy as np

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

        self.lookahead = 1.0 # Need to play around with
        self.speed = 1.0  # FILL IN # Need to play around with
        self.wheelbase_length = 0  # FILL IN #

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

    def pose_callback(self, odometry_msg):

        dx = odometry_msg.twist.twist.linear.x
        dy = odometry_msg.twist.twist.linear.y
        dtheta = odometry_msg.twist.twist.angular.z

        if not self.initialized_traj:
            return

        car_pos = np.array([dx, dy])
        closest_point, segment_idx = self.find_nearest_point_on_trajectory(car_pos)
        lookahead_point = self.find_lookahead_point(closest_point, segment_idx)
        lookahead_x = lookahead_point[0]
        lookahead_y = lookahead_point[1]

        translated_x = lookahead_x - car_pos[0]
        translated_y = lookahead_y - car_pos[1]

        local_x = translated_x * np.cos(dtheta) + translated_y * np.sin(dtheta)
        local_y = translated_x * np.sin(dtheta) - translated_y * np.cos(dtheta)

        steering_angle = np.arctan2(2 * local_y * self.wheelbase_length, self.lookahead**2)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)


    def find_nearest_point_on_trajectory(self, pos):
        """
        Uses vectorized NumPy operations to find the nearest point on the
        trajectory to the given position.
        """
        points = np.array(self.trajectory.points)
        if len(points) < 2:
            return None, None

        V = points[:-1]
        W = points[1:]

        VW = W - V
        VP = pos - V

        VW_dot_VP = np.sum(VW * VP, axis=1)
        VW_dot_VW = np.sum(VW * VW, axis=1)
        VW_dot_VW = np.where(VW_dot_VW == 0, 1e-10, VW_dot_VW)

        t = np.clip(VW_dot_VP / VW_dot_VW, 0.0, 1.0)

        closest_points = V + t[:, np.newaxis] * VW

        dist_squared = np.sum((pos - closest_points)**2, axis=1)
        min_idx = np.argmin(dist_squared)

        return closest_points[min_idx], min_idx

    def circle_line_intersection(self, center, radius, p1, p2):
        """
        Finds the intersection points of a circle and a line segment.
        """
        Q = np.array(center)  # center of circle
        P1 = np.array(p1)     # start of line segment
        P2 = np.array(p2)     # end of line segment
        V = P2 - P1
        a = np.dot(V, V)
        b = 2.0 * np.dot(V, P1 - Q)
        c = np.dot(P1 - Q, P1 - Q) - radius**2

        if a == 0.0:
            return None

        discriminant = b**2 - 4*a*c
        if discriminant < 0.0:
            return None

        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b + sqrt_discriminant) / (2*a)
        t2 = (-b - sqrt_discriminant) / (2*a)

        valid_ts = [t for t in (t1, t2) if 0.0 <= t <= 1.0]

        if not valid_ts:
            return None

        t = max(valid_ts)

        intersection = P1 + t * V
        return intersection


    def find_lookahead_point(self, car_pos, segment_idx):
        '''
        Finds the lookahead point on the trajectory.
        '''
        points = np.array(self.trajectory.points)
        for i in range(segment_idx, len(points) - 1):
            p1 = points[i]
            p2 = points[i+1]
            intersection = self.circle_line_intersection(car_pos, self.lookahead, p1, p2)
            if intersection is not None:
                return intersection
        return points[-1]

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
