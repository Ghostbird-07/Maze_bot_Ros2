import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("mazebot")
    urdf = os.path.join(package_share_dir,"urdf","mazebot.urdf")
    return LaunchDescription([

        # Node(
        #     package='mazebot',
        #     executable='talker',
        #     name='Publisher',
        #     output='screen'
        # ),   
        # Node(
        #     package='mazebot',
        #     executable='listener',
        #     name='Listener',
        #     output='screen'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
            ),
            Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            arguments=[urdf]
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])