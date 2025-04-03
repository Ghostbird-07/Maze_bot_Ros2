#!/usr/bin/python3
# -*- coding: utf-8 -*-
import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mazebot').find('mazebot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mazebot.urdf')

    # Load the world file
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mazebot'), 'launch', 'start_world.py'),
        )
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='mazebot_RD',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
    )

    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen'
    )

    # Add a delay to make sure Gazebo is fully loaded
    spawn_mazebot = TimerAction(
        period=20.0,  # Increase delay if needed
        actions=[
            launch_ros.actions.Node(
                package='gazebo_ros',
                name='mazebot_spawner',
                executable='spawn_entity.py',
                arguments=['-entity', 'mazebot', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '3.14'],
                output='screen'
            )
        ]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'),
        start_world,  # Keep loading the world from your original launch
        robot_state_publisher_node,
        static_transform,
        spawn_mazebot,  # Delayed spawn to avoid timeout
        rviz_node  # Added RViz node
    ])
