import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
import math


# Chain order - each robot follows the one ahead
CHAIN = ['robot1', 'robot2', 'robot3', 'robot4']

# Control gains
LINEAR_GAIN  = 0.5   # how fast to move toward target
ANGULAR_GAIN = 1.5   # how fast to turn toward target
MAX_LINEAR   = 0.2   # max forward speed m/s
MAX_ANGULAR  = 1.0   # max turn speed rad/s
STOP_DIST    = 0.25  # stop when this close to target
FOLLOW_DIST  = 0.7   # desired following distance behind leader


class FollowerVelController(Node):
    def __init__(self):
        super().__init__('follower_vel_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to swarm goal to activate
        self.goal_sub = self.create_subscription(
            PoseStamped, '/swarm/goal', self.on_swarm_goal, 10
        )

        # cmd_vel publishers for followers only
        self.cmd_pubs = {}
        for robot in CHAIN[1:]:
            self.cmd_pubs[robot] = self.create_publisher(
                Twist, f'/{robot}/cmd_vel', 10
            )

        self.active = False
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.get_logger().info('follower_vel_controller ready!')

    def on_swarm_goal(self, msg):
        self.active = True
        self.get_logger().info('Swarm active — followers tracking chain')

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

    def control_loop(self):
        if not self.active:
            return

        for i in range(1, len(CHAIN)):
            follower = CHAIN[i]
            leader   = CHAIN[i - 1]

            follower_pose = self.get_robot_pose(follower)
            leader_pose   = self.get_robot_pose(leader)

            if follower_pose is None or leader_pose is None:
                continue

            fx, fy, fyaw = follower_pose
            lx, ly, _    = leader_pose

            # Compute vector from follower to leader
            dx = lx - fx
            dy = ly - fy
            dist = math.sqrt(dx*dx + dy*dy)

            cmd = Twist()

            if dist <= STOP_DIST:
                # Close enough — stop
                self.cmd_pubs[follower].publish(cmd)
                continue

            # Desired following distance — only move if further than FOLLOW_DIST
            if dist <= FOLLOW_DIST:
                self.cmd_pubs[follower].publish(cmd)
                continue

            # Angle to target
            target_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(target_yaw - fyaw)

            # If facing wrong way, turn first
            if abs(yaw_error) > 0.3:
                cmd.angular.z = max(-MAX_ANGULAR,
                                min(MAX_ANGULAR, ANGULAR_GAIN * yaw_error))
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = min(MAX_LINEAR, LINEAR_GAIN * (dist - FOLLOW_DIST))
                cmd.angular.z = max(-MAX_ANGULAR,
                                min(MAX_ANGULAR, ANGULAR_GAIN * yaw_error))

            self.cmd_pubs[follower].publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = FollowerVelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
