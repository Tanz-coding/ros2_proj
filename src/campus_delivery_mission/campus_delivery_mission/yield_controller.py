import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class YieldController(Node):
    def __init__(self) -> None:
        super().__init__('yield_controller')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mission_pause_topic', '/mission_pause')
        self.declare_parameter('obstacle_distance_topic', '/mission/nearest_obstacle')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('resume_distance', 0.60)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.pause_topic = self.get_parameter('mission_pause_topic').get_parameter_value().string_value
        self.distance_topic = self.get_parameter('obstacle_distance_topic').get_parameter_value().string_value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)

        self.paused = False

        self.pause_pub = self.create_publisher(Bool, self.pause_topic, 10)
        self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [
            value for value in msg.ranges
            if not math.isinf(value) and not math.isnan(value) and msg.range_min <= value <= msg.range_max
        ]
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)
        self.distance_pub.publish(Float32(data=float(min_distance)))

        previous = self.paused
        if self.paused:
            if min_distance > self.resume_distance:
                self.paused = False
        else:
            if min_distance < self.stop_distance:
                self.paused = True

        if previous != self.paused:
            state = 'PAUSED' if self.paused else 'RESUMED'
            self.get_logger().info(f'Yield state changed: {state}, nearest={min_distance:.3f}m')

        self.pause_pub.publish(Bool(data=self.paused))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
