import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('autonomous_tb')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    mission_params = os.path.join(package_share, 'config', 'campus_delivery_params.yaml')
    map_file = os.path.join(package_share, 'maps', 'map.yaml')
    world_file = os.path.join(tb3_gazebo_share, 'worlds', 'turtlebot3_house.world')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Set true to start gzclient GUI.',
    )
    use_gui = LaunchConfiguration('use_gui')

    use_sim_time = SetEnvironmentVariable('use_sim_time', 'true')
    tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'tb3', '-topic', 'robot_description', '-x', '-2.0', '-y', '1.25'],
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

    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])
    delayed_yield = TimerAction(period=10.0, actions=[yield_controller])
    delayed_mission = TimerAction(period=10.0, actions=[campus_delivery])

    return LaunchDescription([
        use_gui_arg,
        use_sim_time,
        tb3_model,
        software_gl,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        delayed_nav2,
        delayed_yield,
        delayed_mission,
    ])
