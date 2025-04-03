from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory("mazebot")
    urdf_file = os.path.join(package_share_dir, "urdf", "mazebot.urdf")

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),

        # Wait 5-10 seconds before spawning to ensure Gazebo is fully ready
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "mazebot", "-file", urdf_file],
                    output="screen"
                )
            ],
        ),

        # Start the robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=[urdf_file],
            output="screen"
        )
    ])
