from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('sam_bot_description')
    
    xacro_file = os.path.join(pkg_path, 'src', 'description', 'robot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_path, 'config', 'my_controllers.yaml')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    # Spawners with a small delay
    spawner_diff = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    spawner_joint = TimerAction(
        period=3.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawner_diff,
        spawner_joint,
    ])

