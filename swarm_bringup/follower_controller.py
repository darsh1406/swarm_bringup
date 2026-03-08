import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


# Formation offsets relative to leader (map frame)
# (x_behind, y_side)
FORMATION = {
    'robot2': (-0.8,  0.6),   # left
    'robot3': (-0.8, -0.6),   # right
    'robot4': (-1.6,  0.0),   # center-rear
}

LEADER = 'robot1'
MOVE_THRESHOLD = 0.5  # metres leader must move before followers activate


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # One action client per follower
        self.action_clients = {}
        for robot in FORMATION.keys():
            self.action_clients[robot] = ActionClient(
                self,
                NavigateToPose,
                f'/{robot}/navigate_to_pose'
            )

        # Track last sent goal per follower to avoid spamming
        self.last_goal = {robot: None for robot in FORMATION.keys()}
        self.goal_handles = {robot: None for robot in FORMATION.keys()}

        # Leader start position
        self.leader_start = None
        self.activated = False

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('follower_controller started, waiting for leader to move...')

    def get_robot_pose(self, robot_name):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{robot_name}/base_footprint',
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw from quaternion
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception:
            return None

    def timer_callback(self):
        leader_pose = self.get_robot_pose(LEADER)
        if leader_pose is None:
            return

        lx, ly, lyaw = leader_pose

        # Record leader start position once
        if self.leader_start is None:
            self.leader_start = (lx, ly)
            return

        # Check if leader has moved enough
        dist = math.sqrt(
            (lx - self.leader_start[0]) ** 2 +
            (ly - self.leader_start[1]) ** 2
        )

        if not self.activated:
            if dist > MOVE_THRESHOLD:
                self.activated = True
                self.get_logger().info(
                    f'Leader moved {dist:.2f}m — activating followers'
                )
            else:
                return

        # Send goals to each follower
        for robot, (dx, dy) in FORMATION.items():
            # Rotate offsets by leader yaw
            goal_x = lx + dx * math.cos(lyaw) - dy * math.sin(lyaw)
            goal_y = ly + dx * math.sin(lyaw) + dy * math.cos(lyaw)

            # Only resend if goal changed significantly (>0.15m)
            if self.last_goal[robot] is not None:
                prev_x, prev_y = self.last_goal[robot]
                if math.sqrt((goal_x - prev_x)**2 + (goal_y - prev_y)**2) < 0.15:
                    continue

            self.send_goal(robot, goal_x, goal_y, lyaw)
            self.last_goal[robot] = (goal_x, goal_y)

    def send_goal(self, robot, x, y, yaw):
        if not self.action_clients[robot].wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{robot} action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw back to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.action_clients[robot].send_goal_async(goal_msg)
        self.get_logger().info(f'Sent goal to {robot}: ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
