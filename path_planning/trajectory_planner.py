import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from path_planning.utils import LineTrajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.ndimage import binary_dilation
from scipy.spatial import KDTree


class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            map_qos)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.map = None
        self.dilated_map = None
        self.current_pose = None

    def map_cb(self, msg):
        self.map = msg
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        occupied = (grid > 50) | (grid < 0)
        radius_px = int(0.50 / msg.info.resolution)  # optimized via experiment 4
        r = radius_px
        y, x = np.ogrid[-r:r+1, -r:r+1]
        struct = x**2 + y**2 <= r**2
        self.dilated_map = binary_dilation(occupied, structure=struct)
        self.get_logger().info("Map received and dilated")

    def pose_cb(self, pose):
        self.current_pose = pose.pose.pose

    def goal_cb(self, msg):
        if self.map is None or self.current_pose is None:
            self.get_logger().warn("Map or pose not yet received")
            return
        start = (self.current_pose.position.x, self.current_pose.position.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan_path(start, goal, self.map)

    def world_to_pixel(self, x, y):
        info = self.map.info
        ox, oy, otheta = info.origin.position.x, info.origin.position.y, np.pi
        dx, dy = x - ox, y - oy
        cos_t, sin_t = np.cos(-otheta), np.sin(-otheta)
        rx = cos_t * dx - sin_t * dy
        ry = sin_t * dx + cos_t * dy
        u = int(rx / info.resolution)
        v = int(ry / info.resolution)
        return (u, v)

    def pixel_to_world(self, u, v):
        info = self.map.info
        ox, oy, otheta = info.origin.position.x, info.origin.position.y, np.pi
        rx = u * info.resolution
        ry = v * info.resolution
        cos_t, sin_t = np.cos(otheta), np.sin(otheta)
        x = cos_t * rx - sin_t * ry + ox
        y = sin_t * rx + cos_t * ry + oy
        return (x, y)

    def is_collision_free(self, p1, p2, n_checks=20):
        for i in range(n_checks + 1):
            t = i / n_checks
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            u, v = self.world_to_pixel(x, y)
            h, w = self.dilated_map.shape
            if not (0 <= u < w and 0 <= v < h):
                return False
            if self.dilated_map[v, u]:
                return False
        return True

    def resample_path(self, path, spacing=0.1):
        new_pts = [path[0]]
        dist_acc = 0.0
        for i in range(1, len(path)):
            p0 = np.array(path[i-1])
            p1 = np.array(path[i])
            seg = np.linalg.norm(p1 - p0)
            if seg < 1e-6:
                continue
            direction = (p1 - p0) / seg
            while dist_acc + spacing <= seg:
                new_pts.append((p0 + (dist_acc + spacing) * direction).tolist())
                dist_acc += spacing
            dist_acc = (dist_acc + spacing) - seg
        return new_pts

    def smooth_path(self, path, iterations=3):
        for _ in range(iterations):
            new = []
            for i in range(len(path)-1):
                p0 = np.array(path[i])
                p1 = np.array(path[i+1])
                Q = 0.75*p0 + 0.25*p1
                R = 0.25*p0 + 0.75*p1
                new.extend([Q.tolist(), R.tolist()])
            path = new
        return path


    def plan_path(self, start_point, end_point, map):
        t0 = time.time()

        STEP_SIZE = 3.0  # optimized via experiment 2
        MAX_ITER = 8000
        GOAL_BIAS = 0.10
        GOAL_THRESH = 1.5
        KD_REBUILD_INTERVAL = 100  # rebuild KDTree every N nodes

        # Compute world-space sampling bounds from map corners
        info = map.info
        corners_px = [(0, 0), (info.width, 0), (0, info.height), (info.width, info.height)]
        corners_w = [self.pixel_to_world(u, v) for u, v in corners_px]
        x_min = min(c[0] for c in corners_w)
        x_max = max(c[0] for c in corners_w)
        y_min = min(c[1] for c in corners_w)
        y_max = max(c[1] for c in corners_w)

        nodes = [start_point]
        parent = {0: None}
        kd = KDTree(nodes)

        goal_idx = None

        for i in range(1, MAX_ITER + 1):
            # Rebuild KDTree periodically
            if i % KD_REBUILD_INTERVAL == 0:
                kd = KDTree(nodes)

            # Sample
            if np.random.random() < GOAL_BIAS:
                x_rand = end_point
            else:
                x_rand = (np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max))

            # Nearest node
            dist_nearest, idx_nearest = kd.query(x_rand)
            x_nearest = nodes[idx_nearest]

            # Steer
            if dist_nearest < STEP_SIZE:
                x_new = x_rand
            else:
                dx = x_rand[0] - x_nearest[0]
                dy = x_rand[1] - x_nearest[1]
                x_new = (x_nearest[0] + STEP_SIZE * dx / dist_nearest,
                         x_nearest[1] + STEP_SIZE * dy / dist_nearest)

            if not self.is_collision_free(x_nearest, x_new):
                continue

            # RRT: connect to nearest node only
            new_idx = len(nodes)
            nodes.append(x_new)
            parent[new_idx] = idx_nearest

            # Check goal
            if np.hypot(x_new[0] - end_point[0], x_new[1] - end_point[1]) < GOAL_THRESH:
                goal_idx = new_idx
                break

        elapsed = time.time() - t0
        self.get_logger().info(
            f"RRT done: {elapsed:.2f}s, {len(nodes)} nodes, success={goal_idx is not None}"
        )

        if goal_idx is None:
            kd_final = KDTree(nodes)
            _, goal_idx = kd_final.query(end_point)

        # Reconstruct path
        path = []
        idx = goal_idx
        while idx is not None:
            path.append(nodes[idx])
            idx = parent[idx]
        path.reverse()

        path = self.resample_path(path, spacing=0.1)
        path = self.smooth_path(path)

        # Publish
        self.trajectory.clear()
        for pt in path:
            self.trajectory.addPoint(pt)
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()



def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
