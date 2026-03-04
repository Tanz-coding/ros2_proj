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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from action_msgs.msg import GoalStatus
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


class CampusDeliveryNode(Node):
    def __init__(self) -> None:
        super().__init__('campus_delivery_node')

        self.declare_parameter('waypoint_names',
                               descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter('waypoint_x',
                               descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('waypoint_y',
                               descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('waypoint_yaw',
                               descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('mission_pause_topic', '/mission_pause')
        self.declare_parameter('obstacle_distance_topic', '/mission/nearest_obstacle')
        self.declare_parameter('output_dir', '~/ros2_proj_data')
        self.declare_parameter('csv_filename', 'delivery_log.csv')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')

        self.waypoints = self._load_waypoints()
        self.image_topic = self.get_parameter('image_topic').value
        self.pause_topic = self.get_parameter('mission_pause_topic').value
        self.obstacle_distance_topic = self.get_parameter('obstacle_distance_topic').value
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.csv_filename = self.get_parameter('csv_filename').value
        self.nav_action_name = self.get_parameter('nav_action_name').value

        self.photos_dir = os.path.join(self.output_dir, 'photos')
        self.logs_dir = os.path.join(self.output_dir, 'logs')
        os.makedirs(self.photos_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        self.csv_path = os.path.join(self.logs_dir, self.csv_filename)
        self._ensure_csv_header()

        self.bridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_obstacle_distance = float('nan')
        self.paused = False
        self.waiting_for_result = False

        self.current_waypoint_index = 0
        self.goal_start_time: Optional[datetime] = None
        self.goal_handle = None
        self.retry_timer = None

        self.navigate_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.create_subscription(Bool, self.pause_topic, self._pause_callback, 10)
        self.create_subscription(Float32, self.obstacle_distance_topic, self._obstacle_callback, 10)

        self.start_timer = self.create_timer(1.0, self._start_if_ready)

    def _load_waypoints(self) -> List[Waypoint]:
        names = self.get_parameter('waypoint_names').value or []
        xs = self.get_parameter('waypoint_x').value or []
        ys = self.get_parameter('waypoint_y').value or []
        yaws = self.get_parameter('waypoint_yaw').value or []

        count = min(len(names), len(xs), len(ys), len(yaws))
        if count == 0:
            self.get_logger().warn('No waypoints configured (arrays empty).')
            return []

        result: List[Waypoint] = []
        for i in range(count):
            result.append(
                Waypoint(
                    name=str(names[i]),
                    x=float(xs[i]),
                    y=float(ys[i]),
                    yaw=float(yaws[i]),
                )
            )
        return result

    def _start_if_ready(self) -> None:
        if not self.waypoints:
            self.get_logger().error('No waypoints configured. Stop mission startup.')
            self.start_timer.cancel()
            return

        if not self.navigate_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...')
            self.navigate_client.wait_for_server(timeout_sec=1.0)
            return

        self.start_timer.cancel()
        self.get_logger().info(f'Campus delivery mission started: {len(self.waypoints)} waypoints.')
        self._send_next_goal_if_possible()

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def _pause_callback(self, msg: Bool) -> None:
        previous = self.paused
        self.paused = bool(msg.data)

        if self.paused and not previous:
            self.get_logger().warn('Mission paused by yield controller.')
            if self.goal_handle is not None and self.waiting_for_result:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self.get_logger().info('Active goal canceled due to pause.'))

        if (not self.paused) and previous:
            self.get_logger().info('Mission resumed.')
            if not self.waiting_for_result:
                self._send_next_goal_if_possible()

    def _obstacle_callback(self, msg: Float32) -> None:
        self.latest_obstacle_distance = float(msg.data)

    def _send_next_goal_if_possible(self) -> None:
        if self.paused:
            self.get_logger().info('Paused state active, postponing goal dispatch.')
            return
        if self.waiting_for_result:
            return
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed.')
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(waypoint)

        self.goal_start_time = datetime.now()
        self.waiting_for_result = True

        self.get_logger().info(
            f'Sending waypoint [{waypoint.name}] ({waypoint.x:.2f}, {waypoint.y:.2f}, yaw={waypoint.yaw:.2f})'
        )
        send_goal_future = self.navigate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.waiting_for_result = False
            self.get_logger().error('Goal rejected by Nav2. Will retry after 2s.')
            if self.retry_timer is None:
                self.retry_timer = self.create_timer(2.0, self._retry_once)
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _retry_once(self) -> None:
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        if not self.waiting_for_result and not self.paused and self.current_waypoint_index < len(self.waypoints):
            self._send_next_goal_if_possible()

    def _on_goal_result(self, future) -> None:
        self.waiting_for_result = False
        result = future.result()
        status = result.status

        waypoint = self.waypoints[self.current_waypoint_index]
        if status == GoalStatus.STATUS_SUCCEEDED:
            photo_path = self._save_photo(waypoint.name)
            self._write_log(waypoint.name, 'succeeded', photo_path)
            self.get_logger().info(f'Arrived at waypoint [{waypoint.name}]')
            self.current_waypoint_index += 1
            self.goal_handle = None
            self._send_next_goal_if_possible()
            return

        if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            self._write_log(waypoint.name, 'paused_or_canceled', '')
            self.get_logger().warn(f'Waypoint [{waypoint.name}] canceled due to pause or external request.')
            self.goal_handle = None
            return

        self._write_log(waypoint.name, f'failed_{status}', '')
        self.get_logger().error(f'Waypoint [{waypoint.name}] failed, status={status}')
        self.current_waypoint_index += 1
        self.goal_handle = None
        self._send_next_goal_if_possible()

    def _build_pose(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return pose

    def _save_photo(self, waypoint_name: str) -> str:
        if self.latest_image is None:
            return ''

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        photo_path = os.path.join(self.photos_dir, f'{waypoint_name}_{timestamp}.jpg')
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            cv2.imwrite(photo_path, image)
            return photo_path
        except Exception as exception:
            self.get_logger().error(f'Failed to save photo: {exception}')
            return ''

    def _ensure_csv_header(self) -> None:
        if os.path.exists(self.csv_path):
            return

        with open(self.csv_path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
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

        obstacle_str = ''
        if not math.isnan(self.latest_obstacle_distance):
            obstacle_str = f'{self.latest_obstacle_distance:.3f}'

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                datetime.now().isoformat(timespec='seconds'),
                waypoint_name,
                status,
                f'{travel_time:.2f}',
                obstacle_str,
                photo_path,
            ])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CampusDeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
