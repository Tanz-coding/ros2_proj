import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('autonomous_tb')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(package_share, 'maps', 'map.yaml')
    default_nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_mission_params = os.path.join(package_share, 'config', 'campus_delivery_params.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    nav2_params_arg = DeclareLaunchArgument('nav2_params', default_value=default_nav2_params)
    mission_params_arg = DeclareLaunchArgument('mission_params', default_value=default_mission_params)
    slam_arg = DeclareLaunchArgument('slam', default_value='True',
                                      description='Use SLAM (online mapping) instead of AMCL localization')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params'),
            'autostart': 'true',
        }.items(),
    )

    yield_controller = Node(
        package='autonomous_tb',
        executable='yield_controller_node',
        name='yield_controller',
        output='screen',
        parameters=[LaunchConfiguration('mission_params')],
    )

    campus_delivery = Node(
        package='autonomous_tb',
        executable='campus_delivery_node',
        name='campus_delivery_node',
        output='screen',
        parameters=[LaunchConfiguration('mission_params')],
    )

    delayed_yield = TimerAction(period=4.0, actions=[yield_controller])
    delayed_mission = TimerAction(period=5.0, actions=[campus_delivery])

    return LaunchDescription([
        map_arg,
        nav2_params_arg,
        mission_params_arg,
        slam_arg,
        nav2,
        delayed_yield,
        delayed_mission,
    ])
