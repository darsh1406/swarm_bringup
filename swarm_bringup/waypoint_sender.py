import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time


# Safe waypoints navigating between cylinder gaps
# Cylinders appear in 3x3 grid — staying in clear corridors
WAYPOINTS = [
    (2.5,  0.5),   # waypoint 1
    (2.5, -1.0),   # waypoint 2
    (0.5, -1.0),   # waypoint 3
    (0.5,  1.0),   # waypoint 4
    (2.5,  1.0),   # waypoint 5
]

FOLLOWER_WAIT_SECS = 15.0  # wait for followers to reach formation before next waypoint


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.client = ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose')
        self.waypoint_index = 0
        self.get_logger().info(f'Waypoint sender ready — {len(WAYPOINTS)} waypoints queued')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.waypoint_index >= len(WAYPOINTS):
            self.get_logger().info('All waypoints completed!')
            return

        x, y = WAYPOINTS[self.waypoint_index]
        self.get_logger().info(
            f'Sending waypoint {self.waypoint_index + 1}/{len(WAYPOINTS)}: ({x}, {y})'
        )

        self.client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            return
        self.get_logger().info('Goal accepted — waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1} reached! '
                f'Waiting {FOLLOWER_WAIT_SECS}s for followers...'
            )
            # Wait for followers to catch up before next waypoint
            self.create_timer(FOLLOWER_WAIT_SECS, self.advance_waypoint)
        else:
            self.get_logger().warn(f'Waypoint failed with status {status} — skipping...')
            self.waypoint_index += 1
            self.send_next_waypoint()

    def advance_waypoint(self):
        self.waypoint_index += 1
        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
