import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


SPAWN_POSITIONS = {
    'robot1': (1.5,  0.0),
    'robot2': (1.0,  1.0),
    'robot3': (1.0, -1.2),
    'robot4': (0.8,  0.0),
}

LEADER = 'robot1'
MAX_RETRIES = 5
SEARCH_STEP = 0.2


class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/swarm/goal', self.on_swarm_goal, 10
        )

        self.nav_clients = {}
        for robot in SPAWN_POSITIONS.keys():
            self.nav_clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose'
            )

        self.current_base = dict(SPAWN_POSITIONS)
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_gyaw = 0.0
        self.retry_counts = {r: 0 for r in SPAWN_POSITIONS}

        self.get_logger().info('swarm_controller ready — publish to /swarm/goal!')

    def on_swarm_goal(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Get robot1 actual position
        actual = self.get_robot_pose(LEADER)
        if actual:
            r1x, r1y, _ = actual
            self.current_base[LEADER] = (r1x, r1y)
        else:
            r1x, r1y = self.current_base[LEADER]

        dx = gx - r1x
        dy = gy - r1y

        self.get_logger().info(f'Swarm goal: ({gx:.2f}, {gy:.2f}) delta=({dx:.2f}, {dy:.2f})')

        self.pending_dx = dx
        self.pending_dy = dy
        self.pending_gyaw = gyaw
        self.retry_counts = {r: 0 for r in SPAWN_POSITIONS}

        # Send only robot1 first
        self.send_nav_goal(LEADER, gx, gy, gyaw)

    def send_followers(self):
        """Called after robot1 reaches its goal."""
        actual = self.get_robot_pose(LEADER)
        if actual:
            r1x, r1y, _ = actual
            self.current_base[LEADER] = (r1x, r1y)

        for follower in ['robot2', 'robot3', 'robot4']:
            bx, by = self.current_base[follower]
            tx = bx + self.pending_dx
            ty = by + self.pending_dy
            self.current_base[follower] = (tx, ty)
            self.get_logger().info(f'Sending {follower} to ({tx:.2f}, {ty:.2f})')
            self.send_nav_goal(follower, tx, ty, self.pending_gyaw)

    def get_spiral_candidate(self, x, y, attempt):
        """Get candidate position at given attempt number using spiral."""
        if attempt == 0:
            return x, y
        ring = (attempt - 1) // 8 + 1
        idx  = (attempt - 1) % 8
        angle = idx * math.pi / 4
        return (
            x + ring * SEARCH_STEP * math.cos(angle),
            y + ring * SEARCH_STEP * math.sin(angle)
        )

    def get_robot_pose(self, robot_name):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{robot_name}/base_footprint', rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception:
            return None

    def send_nav_goal(self, robot, x, y, yaw):
        if not self.nav_clients[robot].wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f'{robot} nav server not available')
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

        future = self.nav_clients[robot].send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, r=robot, gx=x, gy=y: self.goal_response_cb(f, r, gx, gy)
        )

    def goal_response_cb(self, future, robot, x, y):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'{robot} goal REJECTED')
            return
        self.get_logger().info(f'{robot} goal accepted → ({x:.2f}, {y:.2f})')
        handle.get_result_async().add_done_callback(
            lambda f, r=robot, gx=x, gy=y: self.goal_result_cb(f, r, gx, gy)
        )

    def goal_result_cb(self, future, robot, x, y):
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'{robot} SUCCESS ({x:.2f}, {y:.2f})')
            if robot == LEADER:
                self.send_followers()
        else:
            self.retry_counts[robot] += 1
            attempt = self.retry_counts[robot]

            if attempt > MAX_RETRIES:
                self.get_logger().warn(f'{robot} gave up after {MAX_RETRIES} retries')
                return

            # Get next spiral candidate from the original target
            bx, by = self.current_base[robot]
            nx, ny = self.get_spiral_candidate(bx, by, attempt)
            self.get_logger().warn(
                f'{robot} FAILED attempt {attempt} — retrying ({nx:.2f}, {ny:.2f})'
            )
            self.send_nav_goal(robot, nx, ny, self.pending_gyaw)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
