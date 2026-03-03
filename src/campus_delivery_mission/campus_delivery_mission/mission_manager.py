import csv
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')
        self.declare_parameter('waypoints', [])
        self.declare_parameter('mission_pause_topic', '/mission_pause')
        self.declare_parameter('obstacle_distance_topic', '/mission/nearest_obstacle')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '~/ros2_proj_data')
        self.declare_parameter('csv_filename', 'delivery_log.csv')

        self.waypoints = self._load_waypoints()
        self.pause_topic = self.get_parameter('mission_pause_topic').get_parameter_value().string_value
        self.obstacle_distance_topic = self.get_parameter('obstacle_distance_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_root = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(output_root)
        self.csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value

        self.photos_dir = os.path.join(self.output_dir, 'photos')
        self.logs_dir = os.path.join(self.output_dir, 'logs')
        os.makedirs(self.photos_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        self.csv_path = os.path.join(self.logs_dir, self.csv_filename)
        self._ensure_csv_header()

        self.bridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_min_distance = float('nan')
        self.paused = False
        self.current_index = 0
        self.active_goal_handle = None
        self.goal_start_time: Optional[datetime] = None
        self.last_goal_future: Optional[Future] = None

        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(Bool, self.pause_topic, self._pause_callback, 10)
        self.create_subscription(Float32, self.obstacle_distance_topic, self._distance_callback, 10)
        self.create_subscription(Image, self.image_topic, self._image_callback, 10)

        self.start_timer = self.create_timer(1.0, self._start_once)

    def _start_once(self) -> None:
        if not self.waypoints:
            self.get_logger().error('No waypoints configured, mission stopped.')
            self.start_timer.cancel()
            return
        if not self.navigate_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...')
            self.navigate_client.wait_for_server(timeout_sec=1.0)
            return
        self.start_timer.cancel()
        self.get_logger().info(f'Mission started with {len(self.waypoints)} waypoints.')
        self._send_current_goal()

    def _load_waypoints(self) -> List[Waypoint]:
        raw = self.get_parameter('waypoints').value
        result: List[Waypoint] = []
        for item in raw:
            result.append(
                Waypoint(
                    name=str(item.get('name', f'wp_{len(result)}')),
                    x=float(item.get('x', 0.0)),
                    y=float(item.get('y', 0.0)),
                    yaw=float(item.get('yaw', 0.0)),
                )
            )
        return result

    def _pause_callback(self, msg: Bool) -> None:
        was_paused = self.paused
        self.paused = msg.data
        if self.paused and not was_paused and self.active_goal_handle is not None:
            self.get_logger().warn('Obstacle close: pausing mission and canceling current goal.')
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.get_logger().info('Current goal canceled.'))
        if (not self.paused) and was_paused:
            self.get_logger().info('Obstacle cleared: resuming mission.')
            if self.active_goal_handle is None and self.current_index < len(self.waypoints):
                self._send_current_goal()

    def _distance_callback(self, msg: Float32) -> None:
        self.latest_min_distance = float(msg.data)

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def _send_current_goal(self) -> None:
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('Mission complete.')
            return
        if self.paused:
            self.get_logger().info('Mission paused; waiting before sending next goal.')
            return

        waypoint = self.waypoints[self.current_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(waypoint)
        self.goal_start_time = datetime.now()

        send_future = self.navigate_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)
        self.last_goal_future = send_future

        self.get_logger().info(
            f'Sending goal [{waypoint.name}] x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f}'
        )

    def _goal_response_callback(self, future: Future) -> None:
        self.active_goal_handle = future.result()
        if self.active_goal_handle is None or not self.active_goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2, retrying in 2s.')
            self.active_goal_handle = None
            self.create_timer(2.0, self._retry_once)
            return
        result_future = self.active_goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _retry_once(self) -> None:
        if self.active_goal_handle is None and not self.paused and self.current_index < len(self.waypoints):
            self._send_current_goal()

    def _goal_result_callback(self, future: Future) -> None:
        goal_handle = self.active_goal_handle
        self.active_goal_handle = None
        if goal_handle is None:
            return

        status = future.result().status
        waypoint = self.waypoints[self.current_index]

        if status == 4:
            self.get_logger().warn(f'Goal [{waypoint.name}] canceled.')
            self._write_log(waypoint.name, 'canceled', '')
            return
        if status != 4 and status != 2 and status != 6:
            photo_path = self._save_photo(waypoint.name)
            self._write_log(waypoint.name, 'succeeded', photo_path)
            self.get_logger().info(f'Arrived at [{waypoint.name}]')
            self.current_index += 1
            self._send_current_goal()
            return

        self.get_logger().error(f'Goal [{waypoint.name}] failed with status {status}.')
        self._write_log(waypoint.name, f'failed_{status}', '')
        self.current_index += 1
        self._send_current_goal()

    def _build_pose(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0

        qz = math.sin(waypoint.yaw / 2.0)
        qw = math.cos(waypoint.yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _save_photo(self, waypoint_name: str) -> str:
        if self.latest_image is None:
            return ''
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{waypoint_name}_{stamp}.jpg'
        path = os.path.join(self.photos_dir, filename)
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            cv2.imwrite(path, image)
            return path
        except Exception as exc:
            self.get_logger().error(f'Failed to save photo: {exc}')
            return ''

    def _ensure_csv_header(self) -> None:
        if os.path.exists(self.csv_path):
            return
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                'timestamp',
                'waypoint',
                'status',
                'travel_time_sec',
                'nearest_obstacle_m',
                'photo_path',
            ])

    def _write_log(self, waypoint_name: str, status: str, photo_path: str) -> None:
        travel_time = 0.0
        if self.goal_start_time is not None:
            travel_time = (datetime.now() - self.goal_start_time).total_seconds()
        with open(self.csv_path, 'a', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                datetime.now().isoformat(timespec='seconds'),
                waypoint_name,
                status,
                f'{travel_time:.2f}',
                f'{self.latest_min_distance:.3f}' if not math.isnan(self.latest_min_distance) else '',
                photo_path,
            ])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
