import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    package_name = 'sam_bot_description'

    default_world_path = os.path.join(pkg_share, 'world', 'my_world.world')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    # The controller_params are now loaded by the Gazebo plugin, not this launch file directly for the controller manager.
    # However, you might still need the path for the plugin in your URDF.

    # ✅ Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ✅ Start Gazebo
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world')
        ],
        output='screen'
    )

    # ✅ RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # ✅ Twist Mux
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # ❌ REMOVED: The standalone controller manager node is not needed with gazebo_ros2_control
    # control_node = Node(...)

    # ✅ Spawn the robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # ✅ Spawners for controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world_path,
                              description='Full path to world file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),

        rsp,
        gazebo_process,
        # control_node,      # ❌ REMOVED from the list of nodes to launch
        twist_mux,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz_node
    ])
