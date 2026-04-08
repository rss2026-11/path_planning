# import rclpy
# import numpy as np

# from geometry_msgs.msg import PoseArray, PoseStamped
# from nav_msgs.msg import OccupancyGrid, Odometry
# from path_planning.utils import LineTrajectory
# from rclpy.node import Node
# from heapq import heappush, heappop
# from scipy import ndimage


# class PathPlan(Node):

#     def __init__(self):
#         super().__init__("trajectory_planner")
#         self.declare_parameter('odom_topic', "default")
#         self.declare_parameter('map_topic', "default")

#         self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
#         self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

#         self.map_sub = self.create_subscription(
#             OccupancyGrid, self.map_topic, self.map_cb, 1)

#         self.goal_sub = self.create_subscription(
#             PoseStamped, "/goal_pose", self.goal_cb, 10)

#         self.traj_pub = self.create_publisher(
#             PoseArray, "/trajectory/current", 10)

#         self.pose_sub = self.create_subscription(
#             Odometry, self.odom_topic, self.pose_cb, 10)

#         self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

#         self.map_data = None
#         self.map_info = None
#         self.car_pose = None

#     def map_cb(self, msg):
#         self.map_info = msg.info
#         raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
#         occupied = (raw > 50) | (raw < 0)
#         structure = ndimage.generate_binary_structure(2, 2)
#         self.map_data = ndimage.binary_dilation(
#             occupied, structure=structure, iterations=6
#         ).astype(np.int8)
#         self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}")

#     def pose_cb(self, msg):
#         self.car_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

#     def goal_cb(self, msg):
#         if self.map_data is None:
#             self.get_logger().warn("No map received yet")
#             return
#         if self.car_pose is None:
#             self.get_logger().warn("No car pose received yet")
#             return

#         goal = (msg.pose.position.x, msg.pose.position.y)
#         self.get_logger().info(f"Planning from {self.car_pose} to {goal}")
#         self.plan_path(self.car_pose, goal, self.map_data)

#     def world_to_grid(self, x, y):
#         res = self.map_info.resolution
#         origin = self.map_info.origin
#         qz = origin.orientation.z
#         qw = origin.orientation.w
#         yaw = 2.0 * np.arctan2(qz, qw)
#         cos_yaw = np.cos(yaw)
#         sin_yaw = np.sin(yaw)
#         dx = x - origin.position.x
#         dy = y - origin.position.y
#         u = int((cos_yaw * dx + sin_yaw * dy) / res)
#         v = int((-sin_yaw * dx + cos_yaw * dy) / res)
#         return u, v

#     def grid_to_world(self, u, v):
#         res = self.map_info.resolution
#         origin = self.map_info.origin
#         qz = origin.orientation.z
#         qw = origin.orientation.w
#         yaw = 2.0 * np.arctan2(qz, qw)
#         cos_yaw = np.cos(yaw)
#         sin_yaw = np.sin(yaw)
#         x = origin.position.x + res * (cos_yaw * u - sin_yaw * v)
#         y = origin.position.y + res * (sin_yaw * u + cos_yaw * v)
#         return x, y

#     def heuristic(self, a, b):
#         return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

#     def plan_path(self, start_point, end_point, map):
#         start_u, start_v = self.world_to_grid(start_point[0], start_point[1])
#         end_u, end_v = self.world_to_grid(end_point[0], end_point[1])

#         if self.map_data[start_v, start_u] != 0:
#             self.get_logger().warn("Start is inside an obstacle!")
#             return
#         if self.map_data[end_v, end_u] != 0:
#             self.get_logger().warn("Goal is inside an obstacle!")
#             return

#         open_set = []
#         came_from = {}
#         g_costs = {}
#         visited = set()

#         start = (start_u, start_v)
#         goal = (end_u, end_v)
#         g_costs[start] = 0
#         h = self.heuristic(start, goal)
#         heappush(open_set, (h, start[0], start[1]))

#         while len(open_set) > 0:
#             f, u, v = heappop(open_set)
#             current = (u, v)

#             if current in visited:
#                 continue
#             visited.add(current)

#             if current == goal:
#                 path = []
#                 node = goal
#                 while node in came_from:
#                     path.append(node)
#                     node = came_from[node]
#                 path.append(start)
#                 path.reverse()

#                 self.trajectory.clear()
#                 for point in path:
#                     x, y = self.grid_to_world(point[0], point[1])
#                     self.trajectory.addPoint((x, y))

#                 self.traj_pub.publish(self.trajectory.toPoseArray())
#                 self.trajectory.publish_viz()
#                 self.get_logger().info(f"Path found with {len(path)} points")
#                 return

#             neighbors = [(-1,-1), (0,-1), (1,-1),
#                          (-1, 0),         (1, 0),
#                          (-1, 1), (0, 1), (1, 1)]

#             for du, dv in neighbors:
#                 nu, nv = u + du, v + dv

#                 if nu < 0 or nu >= self.map_info.width:
#                     continue
#                 if nv < 0 or nv >= self.map_info.height:
#                     continue

#                 if self.map_data[nv, nu] != 0:
#                     continue

#                 if du != 0 and dv != 0:
#                     move_cost = 1.414
#                 else:
#                     move_cost = 1.0

#                 new_g = g_costs[current] + move_cost
#                 neighbor = (nu, nv)

#                 if new_g < g_costs.get(neighbor, float('inf')):
#                     g_costs[neighbor] = new_g
#                     f = new_g + self.heuristic(neighbor, goal)
#                     heappush(open_set, (f, nu, nv))
#                     came_from[neighbor] = current

#         self.get_logger().warn("No path found!")


# def main(args=None):
#     rclpy.init(args=args)
#     planner = PathPlan()
#     rclpy.spin(planner)
#     rclpy.shutdown()








import rclpy
import numpy as np

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from path_planning.utils import LineTrajectory
from rclpy.node import Node
from heapq import heappush, heappop
from scipy import ndimage


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

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

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

        self.map_data = None
        self.map_info = None
        self.car_pose = None

    def map_cb(self, msg):
        self.map_info = msg.info
        raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        occupied = (raw > 50) | (raw < 0)
        structure = ndimage.generate_binary_structure(2, 2)
        self.map_data = ndimage.binary_dilation(occupied, structure=structure, iterations=4).astype(np.int8)
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}")
        self.get_logger().info(f"Map origin: ({msg.info.origin.position.x}, {msg.info.origin.position.y}), resolution: {msg.info.resolution}")
        o = msg.info.origin.orientation
        self.get_logger().info(f"Map orientation: x={o.x}, y={o.y}, z={o.z}, w={o.w}")

    def pose_cb(self, msg):
        self.car_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg):
        if self.map_data is None:
            self.get_logger().warn("No map received yet")
            return
        if self.car_pose is None:
            self.get_logger().warn("No car pose received yet")
            return

        goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Planning from {self.car_pose} to {goal}")
        self.plan_path(self.car_pose, goal, self.map_data)

    def world_to_grid(self, x, y):
        res = self.map_info.resolution
        origin = self.map_info.origin
        # Get yaw from quaternion
        qz = origin.orientation.z
        qw = origin.orientation.w
        yaw = 2.0 * np.arctan2(qz, qw)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        # Translate then rotate
        dx = x - origin.position.x
        dy = y - origin.position.y
        # Inverse rotation
        grid_x = (cos_yaw * dx + sin_yaw * dy) / res
        grid_y = (-sin_yaw * dx + cos_yaw * dy) / res
        u = int(grid_x)
        v = int(grid_y)
        return u, v

    def grid_to_world(self, u, v):
        res = self.map_info.resolution
        origin = self.map_info.origin
        # Get yaw from quaternion
        qz = origin.orientation.z
        qw = origin.orientation.w
        yaw = 2.0 * np.arctan2(qz, qw)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        # Forward rotation
        x = origin.position.x + res * (cos_yaw * u - sin_yaw * v)
        y = origin.position.y + res * (sin_yaw * u + cos_yaw * v)
        return x, y

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def plan_path(self, start_point, end_point, map):
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1])
        end_u, end_v = self.world_to_grid(end_point[0], end_point[1])

        # self.get_logger().info(f"Start grid: ({start_u}, {start_v}), value: {self.map_data[start_v, start_u]}")
        # self.get_logger().info(f"Goal grid: ({end_u}, {end_v}), value: {self.map_data[end_v, end_u]}")

        if self.map_data[start_v, start_u] != 0:
            self.get_logger().warn("Start is inside an obstacle!")
            return
        if self.map_data[end_v, end_u] != 0:
            self.get_logger().warn("Goal is inside an obstacle!")
            return

        open_set = []
        came_from = {}
        g_costs = {}
        visited = set()

        start = (start_u, start_v)
        goal = (end_u, end_v)
        g_costs[start] = 0
        h = self.heuristic(start, goal)
        heappush(open_set, (h, start[0], start[1]))

        while len(open_set) > 0:
            f, u, v = heappop(open_set)
            current = (u, v)
            
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()

                self.trajectory.clear()
                for point in path:
                    x, y = self.grid_to_world(point[0], point[1])
                    self.trajectory.addPoint((x, y))

                self.traj_pub.publish(self.trajectory.toPoseArray())
                self.trajectory.publish_viz()
                return

            neighbors = [(-1,-1), (0,-1), (1,-1),
                         (-1, 0),         (1, 0),
                         (-1, 1), (0, 1), (1, 1)]

            for du, dv in neighbors:
                nu, nv = u + du, v + dv

                if nu < 0 or nu >= self.map_info.width:
                    continue
                if nv < 0 or nv >= self.map_info.height:
                    continue

                if self.map_data[nv, nu] != 0:
                    continue

                if du != 0 and dv != 0:
                    move_cost = np.sqrt(2)
                else:
                    move_cost = 1.0

                new_g = g_costs[current] + move_cost
                neighbor = (nu, nv)

                if new_g < g_costs.get(neighbor, float('inf')):
                    g_costs[neighbor] = new_g
                    f = new_g + self.heuristic(neighbor, goal)
                    heappush(open_set, (f, nu, nv))
                    came_from[neighbor] = current

        self.get_logger().warn("No path found!")


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()