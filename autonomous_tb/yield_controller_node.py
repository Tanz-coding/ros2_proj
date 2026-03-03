import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class YieldControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('yield_controller_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mission_pause_topic', '/mission_pause')
        self.declare_parameter('obstacle_distance_topic', '/mission/nearest_obstacle')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('resume_distance', 0.60)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.pause_topic = self.get_parameter('mission_pause_topic').value
        self.obstacle_distance_topic = self.get_parameter('obstacle_distance_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)

        self.paused = False

        self.pause_publisher = self.create_publisher(Bool, self.pause_topic, 10)
        self.distance_publisher = self.create_publisher(Float32, self.obstacle_distance_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)

        self.get_logger().info(
            f'Yield controller started: stop_distance={self.stop_distance:.2f}, resume_distance={self.resume_distance:.2f}'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        valid = [
            value
            for value in msg.ranges
            if not math.isnan(value) and not math.isinf(value) and msg.range_min <= value <= msg.range_max
        ]
        if not valid:
            return

        nearest = min(valid)
        self.distance_publisher.publish(Float32(data=float(nearest)))

        previous = self.paused
        if self.paused:
            if nearest > self.resume_distance:
                self.paused = False
        else:
            if nearest < self.stop_distance:
                self.paused = True

        if previous != self.paused:
            state = 'PAUSED' if self.paused else 'RESUMED'
            self.get_logger().warn(f'Yield state changed: {state}, nearest={nearest:.3f}m')

        self.pause_publisher.publish(Bool(data=self.paused))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YieldControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
