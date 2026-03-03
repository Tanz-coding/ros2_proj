from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('campus_delivery_mission'),
        'config',
        'waypoints.yaml',
    ])

    yield_node = Node(
        package='campus_delivery_mission',
        executable='yield_controller',
        name='yield_controller',
        output='screen',
        parameters=[params_file],
    )

    mission_node = Node(
        package='campus_delivery_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([yield_node, mission_node])
