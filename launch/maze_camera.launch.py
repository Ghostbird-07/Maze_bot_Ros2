import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("mazebot")
    urdf_file = os.path.join(package_share_dir, "urdf", "mazebot.urdf")
    world_file = os.path.join(package_share_dir, "world", "mazeworld.world")
    
    # Properly set Gazebo paths
    env = {
        "GAZEBO_MODEL_PATH": os.path.join(get_package_share_directory("mazebot"), "models") + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
        "GAZEBO_PLUGIN_PATH": os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        "GAZEBO_RESOURCE_PATH": os.path.join(get_package_share_directory("mazebot")) + ":" + os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        # Force software rendering
        "LIBGL_ALWAYS_SOFTWARE": "1",
        # For debugging
        "GAZEBO_DEBUG": "1"
    }
    
    return LaunchDescription([
        # Use gzserver instead of gazebo (headless)
        ExecuteProcess(
            cmd=["gzserver", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
            output="screen",
            additional_env=env,
        ),
        
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[urdf_file],
        ),
    ])