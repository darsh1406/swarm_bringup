import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import math


# For each robot, filter out these other robots from its scan
FILTER_MAP = {
    'robot1': ['robot2', 'robot3', 'robot4'],
    'robot2': ['robot1', 'robot3', 'robot4'],
    'robot3': ['robot1', 'robot2', 'robot4'],
    'robot4': ['robot1', 'robot2', 'robot3'],
}

ROBOT_RADIUS = 0.25  # filter points within this radius of another robot


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.others = FILTER_MAP[self.robot_name]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.on_scan,
            10
        )
        self.pub = self.create_publisher(
            LaserScan,
            f'/{self.robot_name}/scan_filtered',
            10
        )
        self.get_logger().info(
            f'scan_filter started for {self.robot_name}, filtering: {self.others}'
        )

    def get_robot_pos(self, robot_name):
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self.robot_name}/base_scan',
                f'{robot_name}/base_footprint',
                rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def on_scan(self, msg):
        # Get positions of all other robots in this robot's scan frame
        other_positions = []
        for other in self.others:
            pos = self.get_robot_pos(other)
            if pos is not None:
                other_positions.append(pos)

        filtered_ranges = list(msg.ranges)

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            px = r * math.cos(angle)
            py = r * math.sin(angle)

            for ox, oy in other_positions:
                dist = math.sqrt((px - ox)**2 + (py - oy)**2)
                if dist < ROBOT_RADIUS:
                    filtered_ranges[i] = float('inf')
                    break

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities

        self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
