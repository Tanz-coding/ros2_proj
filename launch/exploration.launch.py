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
    # Core package directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Project files
    config_dir = os.path.join(get_package_share_directory('autonomous_tb'), 'config')
    world_file = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_house.world')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Set true to start gzclient GUI.'
    )
    use_gui = LaunchConfiguration('use_gui')

    # Environment
    use_sim_time = SetEnvironmentVariable('use_sim_time', 'true')
    tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # Gazebo server (stable headless default)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'tb3', '-topic', 'robot_description', '-x', '-2.0', '-y', '1.25'],
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_cartographer'), 'launch'),
            '/cartographer.launch.py']),
        launch_arguments={'use_sim_time': 'true',
                          'log_level': 'warn'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'bringup_launch.py')]),
        launch_arguments={
            'map': 'none',
            'use_sim_time': 'true',
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
            'autostart': 'true'
        }.items(),
    )

    # Launch the exploration node
    exploration_node = Node(
        package='autonomous_tb',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_cartographer = TimerAction(period=6.0, actions=[cartographer])
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])
    delayed_exploration = TimerAction(period=10.0, actions=[exploration_node])

    ld = LaunchDescription()

    ld.add_action(use_gui_arg)
    ld.add_action(use_sim_time)
    ld.add_action(tb3_model)
    ld.add_action(software_gl)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(delayed_cartographer)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_exploration)

    return ld