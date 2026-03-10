import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


FORMATION = {
    'robot2': (-0.7,  0.2),
    'robot4': (-2.0,  0.0),
    'robot3': (-1.2,  0.0),
}

LEADER = 'robot1'
SEARCH_STEP = 0.15
SEARCH_RINGS = 10


class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/swarm/goal', self.on_swarm_goal, 10
        )

        self.nav_clients = {}
        self.path_clients = {}
        for robot in [LEADER] + list(FORMATION.keys()):
            self.nav_clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose'
            )
        for robot in FORMATION.keys():
            self.path_clients[robot] = ActionClient(
                self, ComputePathToPose, f'/{robot}/compute_path_to_pose'
            )

        # Queue for sequential follower dispatch
        self.pending_followers = []
        self.dispatching = False

        self.get_logger().info('swarm_controller ready — publish to /swarm/goal!')

    def on_swarm_goal(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(f'Swarm goal: ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})')

        # Send robot1 immediately
        self.send_nav_goal(LEADER, gx, gy, gyaw)

        # Queue followers for sequential dispatch
        self.pending_followers = []
        for robot, (dx, dy) in FORMATION.items():
            fx = gx + dx * math.cos(gyaw) - dy * math.sin(gyaw)
            fy = gy + dx * math.sin(gyaw) + dy * math.cos(gyaw)
            self.pending_followers.append((robot, fx, fy, gyaw))

        # Start dispatching after 5s delay
        self.dispatching = True
        self._dispatch_timer = self.create_timer(5.0, self._dispatch_next)

    def _dispatch_next(self):
        self._dispatch_timer.cancel()
        if not self.pending_followers:
            self.dispatching = False
            return

        robot, fx, fy, gyaw = self.pending_followers.pop(0)
        candidates = self.generate_candidates(fx, fy)
        self.try_next_candidate(robot, candidates, 0, gyaw)

        # Schedule next follower after 10s
        if self.pending_followers:
            self._dispatch_timer = self.create_timer(10.0, self._dispatch_next)

    def generate_candidates(self, x, y):
        candidates = [(x, y)]
        for ring in range(1, SEARCH_RINGS + 1):
            n_points = max(8, ring * 8)
            for i in range(n_points):
                angle = 2 * math.pi * i / n_points
                nx = x + ring * SEARCH_STEP * math.cos(angle)
                ny = y + ring * SEARCH_STEP * math.sin(angle)
                candidates.append((nx, ny))
        return candidates

    def try_next_candidate(self, robot, candidates, idx, yaw):
        if idx >= len(candidates):
            self.get_logger().warn(f'{robot} no free position found!')
            return

        cx, cy = candidates[idx]

        if not self.path_clients[robot].wait_for_server(timeout_sec=1.0):
            self.send_nav_goal(robot, cx, cy, yaw)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = cx
        goal_msg.goal.pose.position.y = cy
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = False

        future = self.path_clients[robot].send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, r=robot, c=candidates, i=idx, yw=yaw, x=cx, y=cy:
                self.path_response_cb(f, r, c, i, yw, x, y)
        )

    def path_response_cb(self, future, robot, candidates, idx, yaw, cx, cy):
        handle = future.result()
        if not handle.accepted:
            self.try_next_candidate(robot, candidates, idx + 1, yaw)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=robot, c=candidates, i=idx, yw=yaw, x=cx, y=cy:
                self.path_result_cb(f, r, c, i, yw, x, y)
        )

    def path_result_cb(self, future, robot, candidates, idx, yaw, cx, cy):
        result = future.result()
        if len(result.result.path.poses) > 0:
            ox, oy = candidates[0]
            if cx != ox or cy != oy:
                self.get_logger().info(
                    f'{robot} shifted goal from ({ox:.2f},{oy:.2f}) to ({cx:.2f},{cy:.2f})'
                )
            self.send_nav_goal(robot, cx, cy, yaw)
        else:
            self.try_next_candidate(robot, candidates, idx + 1, yaw)

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
        self.get_logger().info(f'Sent nav goal to {robot}: ({x:.2f}, {y:.2f})')

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
    node = SwarmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
