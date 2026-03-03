import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('autonomous_tb')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    mission_params = os.path.join(package_share, 'config', 'campus_delivery_params.yaml')
    map_file = os.path.join(package_share, 'maps', 'map.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_house.launch.py')
        ),
        launch_arguments={
            'x_pose': '-2.0',
            'y_pose': '1.25',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    yield_controller = Node(
        package='autonomous_tb',
        executable='yield_controller_node',
        name='yield_controller',
        output='screen',
        parameters=[mission_params],
    )

    campus_delivery = Node(
        package='autonomous_tb',
        executable='campus_delivery_node',
        name='campus_delivery_node',
        output='screen',
        parameters=[mission_params],
    )

    return LaunchDescription([
        gazebo,
        nav2,
        yield_controller,
        campus_delivery,
    ])
