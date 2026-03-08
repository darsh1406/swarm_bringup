import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


FORMATION = {
    'robot2': (-0.5,  0.4),
    'robot3': (-0.5, -0.4),
    'robot4': (-1.0,  0.0),
}

LEADER = 'robot1'
OBSTACLE_THRESHOLD = 50  # costmap value above this = obstacle


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to robot1 nav status
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            f'/{LEADER}/navigate_to_pose/_action/status',
            self.on_leader_status,
            10
        )

        # Subscribe to each follower's global costmap
        self.costmaps = {}
        for robot in FORMATION.keys():
            self.create_subscription(
                OccupancyGrid,
                f'/{robot}/global_costmap/costmap',
                lambda msg, r=robot: self.costmap_cb(msg, r),
                rclpy.qos.QoSProfile(
                    depth=1,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
                )
            )
            self.costmaps[robot] = None

        # Action clients for followers
        self.action_clients = {}
        for robot in FORMATION.keys():
            self.action_clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose'
            )

        self.last_status = None
        self.get_logger().info('follower_controller started — waiting for robot1...')

    def costmap_cb(self, msg, robot):
        self.costmaps[robot] = msg

    def get_robot_pose(self, robot_name):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', f'{robot_name}/base_footprint', rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def is_free(self, costmap, x, y):
        """Check if (x, y) in map coords is free in the costmap."""
        if costmap is None:
            return True  # assume free if no costmap yet

        res = costmap.info.resolution
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        w = costmap.info.width
        h = costmap.info.height

        cx = int((x - ox) / res)
        cy = int((y - oy) / res)

        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return False  # out of map bounds

        idx = cy * w + cx
        val = costmap.data[idx]
        return val < OBSTACLE_THRESHOLD and val >= 0

    def find_free_goal(self, robot, x, y):
        """Search outward from (x,y) in spiral to find nearest free cell."""
        costmap = self.costmaps[robot]
        if costmap is None:
            return x, y  # no costmap, use original

        if self.is_free(costmap, x, y):
            return x, y  # already free

        self.get_logger().warn(
            f'{robot} goal ({x:.2f}, {y:.2f}) is in obstacle — searching for free cell...'
        )

        res = costmap.info.resolution
        # Search in expanding rings
        for radius in range(1, 30):
            step = res
            angle_steps = max(8, int(2 * math.pi * radius / step))
            for i in range(angle_steps):
                angle = 2 * math.pi * i / angle_steps
                nx = x + radius * res * math.cos(angle)
                ny = y + radius * res * math.sin(angle)
                if self.is_free(costmap, nx, ny):
                    self.get_logger().info(
                        f'{robot} found free cell at ({nx:.2f}, {ny:.2f})'
                    )
                    return nx, ny

        self.get_logger().warn(f'{robot} no free cell found — using original')
        return x, y

    def on_leader_status(self, msg):
        if not msg.status_list:
            return

        current_status = msg.status_list[-1].status

        if current_status == 4 and self.last_status != 4:
            self.get_logger().info('robot1 reached goal! Sending formation goals...')
            leader_pose = self.get_robot_pose(LEADER)
            if leader_pose is None:
                self.get_logger().warn('Could not get robot1 pose')
                self.last_status = current_status
                return

            lx, ly, lyaw = leader_pose
            self.get_logger().info(f'robot1 at ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f})')

            for robot, (dx, dy) in FORMATION.items():
                goal_x = lx + dx * math.cos(lyaw) - dy * math.sin(lyaw)
                goal_y = ly + dx * math.sin(lyaw) + dy * math.cos(lyaw)

                # Find nearest free position if goal is in obstacle
                goal_x, goal_y = self.find_free_goal(robot, goal_x, goal_y)
                self.send_goal(robot, goal_x, goal_y, lyaw)

        self.last_status = current_status

    def send_goal(self, robot, x, y, yaw):
        if not self.action_clients[robot].wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f'{robot} action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.action_clients[robot].send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, r=robot, gx=x, gy=y: self.goal_response_cb(f, r, gx, gy)
        )
        self.get_logger().info(f'Sent goal to {robot}: ({x:.2f}, {y:.2f})')

    def goal_response_cb(self, future, robot, x, y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{robot} goal REJECTED')
            return
        self.get_logger().info(f'{robot} goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=robot, gx=x, gy=y: self.goal_result_cb(f, r, gx, gy)
        )

    def goal_result_cb(self, future, robot, x, y):
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'{robot} reached ({x:.2f}, {y:.2f}) SUCCESS')
        else:
            self.get_logger().warn(f'{robot} FAILED ({x:.2f}, {y:.2f}) status={status}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
