import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
import struct
import math


ROBOT_PRIORITY = ['robot1', 'robot2', 'robot3', 'robot4']


class ObstacleBroadcaster(Node):
    def __init__(self):
        super().__init__('obstacle_broadcaster')

        self.declare_parameter('robot_name', 'robot2')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Only broadcast higher-priority robots as obstacles
        my_index = ROBOT_PRIORITY.index(self.robot_name)
        self.higher_priority = ROBOT_PRIORITY[:my_index]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/other_robots_obstacles',
            10
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            f'obstacle_broadcaster started for {self.robot_name}, '
            f'tracking: {self.higher_priority}'
        )

    def timer_callback(self):
        points = []

        for other_robot in self.higher_priority:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    f'{other_robot}/base_footprint',
                    rclpy.time.Time()
                )

                cx = transform.transform.translation.x
                cy = transform.transform.translation.y

                # Center point + 8 surrounding points spread ~0.2m
                offsets = [
                    (0.0,   0.0),
                    (0.2,   0.0),
                    (-0.2,  0.0),
                    (0.0,   0.2),
                    (0.0,  -0.2),
                    (0.2,   0.2),
                    (0.2,  -0.2),
                    (-0.2,  0.2),
                    (-0.2, -0.2),
                ]

                for dx, dy in offsets:
                    points.append((cx + dx, cy + dy, 0.0))

            except Exception:
                # TF not yet available — skip this robot silently
                continue

        if not points:
            return

        cloud = self.create_pointcloud2(points)
        self.publisher.publish(cloud)

    def create_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 x float32
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        data = []
        for x, y, z in points:
            data += struct.pack('fff', x, y, z)

        msg.data = bytes(data)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
