import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math


# Formation offsets relative to leader (map frame)
FORMATION = {
    'robot2': (-0.8,  0.6),
    'robot3': (-0.8, -0.6),
    'robot4': (-1.6,  0.0),
}

LEADER = 'robot1'

# Controller gains
KP_LINEAR  = 0.5
KP_ANGULAR = 1.0

# Thresholds
LINEAR_DEADBAND  = 0.05  # m
ANGULAR_DEADBAND = 0.05  # rad
MAX_LINEAR_VEL   = 0.22  # m/s  (TurtleBot3 waffle max)
MAX_ANGULAR_VEL  = 1.82  # rad/s


class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publishers = {}
        for robot in FORMATION.keys():
            self.publishers[robot] = self.create_publisher(
                Twist,
                f'/{robot}/cmd_vel',
                10
            )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('formation_controller started')

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
        except Exception:
            return None

    def clamp(self, value, max_val):
        return max(-max_val, min(max_val, value))

    def timer_callback(self):
        leader_pose = self.get_robot_pose(LEADER)
        if leader_pose is None:
            return

        lx, ly, lyaw = leader_pose

        for robot, (dx, dy) in FORMATION.items():
            follower_pose = self.get_robot_pose(robot)
            if follower_pose is None:
                continue

            fx, fy, fyaw = follower_pose

            # Desired position in map frame
            goal_x = lx + dx * math.cos(lyaw) - dy * math.sin(lyaw)
            goal_y = ly + dx * math.sin(lyaw) + dy * math.cos(lyaw)

            # Error in map frame
            ex = goal_x - fx
            ey = goal_y - fy
            distance = math.sqrt(ex**2 + ey**2)

            # Desired heading toward goal
            desired_yaw = math.atan2(ey, ex)
            yaw_error = desired_yaw - fyaw

            # Normalize yaw error to [-pi, pi]
            while yaw_error >  math.pi: yaw_error -= 2.0 * math.pi
            while yaw_error < -math.pi: yaw_error += 2.0 * math.pi

            cmd = Twist()

            if distance > LINEAR_DEADBAND:
                # Only move forward if roughly facing the goal
                if abs(yaw_error) < math.pi / 2.0:
                    cmd.linear.x = self.clamp(
                        KP_LINEAR * distance, MAX_LINEAR_VEL
                    )
                cmd.angular.z = self.clamp(
                    KP_ANGULAR * yaw_error, MAX_ANGULAR_VEL
                )
            elif abs(yaw_error) > ANGULAR_DEADBAND:
                # In position — just align heading with leader
                leader_yaw_error = lyaw - fyaw
                while leader_yaw_error >  math.pi: leader_yaw_error -= 2.0 * math.pi
                while leader_yaw_error < -math.pi: leader_yaw_error += 2.0 * math.pi
                cmd.angular.z = self.clamp(
                    KP_ANGULAR * leader_yaw_error, MAX_ANGULAR_VEL
                )

            self.publishers[robot].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
