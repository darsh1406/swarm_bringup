import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


FORMATION = {
    'robot2': (-0.8,  0.6),
    'robot3': (-0.8, -0.6),
    'robot4': (-1.6,  0.0),
}

LEADER = 'robot1'


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to robot1's navigate_to_pose status
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            f'/{LEADER}/navigate_to_pose/_action/status',
            self.on_leader_status,
            10
        )

        # Action clients for followers
        self.action_clients = {}
        for robot in FORMATION.keys():
            self.action_clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose'
            )

        self.last_status = None  # track previous status to detect transition
        self.get_logger().info('follower_controller started — waiting for robot1 to complete a goal...')

    def get_robot_pose(self, robot_name):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{robot_name}/base_footprint',
                rclpy.time.Time()
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

    def on_leader_status(self, msg):
        if not msg.status_list:
            return

        # Get the most recent goal's status
        current_status = msg.status_list[-1].status

        # Detect transition TO succeeded (status 4)
        # Only trigger once when it first becomes 4
        if current_status == 4 and self.last_status != 4:
            self.get_logger().info('robot1 reached its goal! Sending formation goals...')
            leader_pose = self.get_robot_pose(LEADER)
            if leader_pose is None:
                self.get_logger().warn('Could not get robot1 pose')
                self.last_status = current_status
                return

            lx, ly, lyaw = leader_pose
            self.get_logger().info(f'robot1 at ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f} rad)')

            for robot, (dx, dy) in FORMATION.items():
                goal_x = lx + dx * math.cos(lyaw) - dy * math.sin(lyaw)
                goal_y = ly + dx * math.sin(lyaw) + dy * math.cos(lyaw)
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
