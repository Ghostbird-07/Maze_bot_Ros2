#!/usr/bin/python3
# -*- coding: utf-8 -*-
import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mazebot').find('mazebot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    
    # Process the XACRO file to get URDF content
    robot_description_content = Command(['xacro ', urdf_file])
    
    # Load the world file
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mazebot'), 'launch', 'start_world.py'),
        )
    )
    
    # Use the processed URDF content for robot_state_publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='mazebot_RD',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                    'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen'
    )
    
    # Generate a temporary URDF file for the spawn entity
    # This is important - we need to process the XACRO file before spawning
    # Create a temporary processed URDF file
    urdf_file_processed = os.path.join(pkg_share, 'urdf', 'my_robot_processed.urdf')
    
    # Add a delay to make sure Gazebo is fully loaded
    spawn_mazebot = TimerAction(
        period=20.0,  # Increase delay if needed
        actions=[
            # Use Command to process XACRO to URDF on-the-fly
            launch_ros.actions.Node(
                package='gazebo_ros',
                name='mazebot_spawner',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'mazebot', 
                    '-topic', '/robot_description',  # Use the topic from robot_state_publisher
                    '-x', '0.0', 
                    '-y', '0.0', 
                    '-z', '0.1', 
                    '-Y', '3.14'
                ],
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
        start_world,
        robot_state_publisher_node,
        static_transform,
        spawn_mazebot,
        rviz_node
    ])