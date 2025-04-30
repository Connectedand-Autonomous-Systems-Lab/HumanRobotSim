import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import subprocess

def record_ros2_bag(bag_name="lidar_experiment"):
    try:
        # Command to record the topics
        # command = ["ros2", "bag", "record", "-o", bag_name, "/gaze/point", "/human/scan", "/tf", "/scan"]
        command = ["ros2", "bag", "record", "-o", bag_name, "/human/scan", "/tf"]

        # Launch the process
        process = subprocess.Popen(command)

        print(f"Recording ROS 2 bag: {bag_name}")
        print("Press Ctrl+C to stop recording.")

        # Wait for the process to complete (user stops it manually)
        process.wait()

    except KeyboardInterrupt:
        print("\nStopping bag recording...")
        process.terminate()
        
if __name__=="__main__":
    record_ros2_bag()
