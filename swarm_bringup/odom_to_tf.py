import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.spawn_x = self.get_parameter('spawn_x').get_parameter_value().double_value
        self.spawn_y = self.get_parameter('spawn_y').get_parameter_value().double_value

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.published = False

        self.sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'odom_to_tf waiting for first odom from /{self.robot_name}/odom ...'
        )

    def odom_callback(self, msg):
        if self.published:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.robot_name}/odom'

        t.transform.translation.x = self.spawn_x
        t.transform.translation.y = self.spawn_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
        self.published = True

        self.get_logger().info(
            f'Published static TF: map -> {self.robot_name}/odom '
            f'at ({self.spawn_x}, {self.spawn_y})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
