import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    pause = LaunchConfiguration("pause", default="false")
    world_file_name = "turtlebot3_drl/" + "waffle" + ".model"
    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "worlds", world_file_name
    )
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_turtlebot3_cartographer = get_package_share_directory("turtlebot3_cartographer")
    pkg_turtlebot3_slam = get_package_share_directory("slam_toolbox")


    return LaunchDescription(
        [
            # Gazebo server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world, "pause": pause}.items(),
            ),
            # Gazebo client
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                ),
            ),
            # Robot state publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_file_dir, "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),

            # Cartographer
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(pkg_turtlebot3_cartographer, "launch", "cartographer.launch.py")
            #     ),
            #     launch_arguments={"use_sim_time": use_sim_time, "rviz_config_dir": "/home/mayooran/Documents/DRL-Robot-Navigation-ROS2/src/turtlebot3_simulations/turtlebot3_gazebo/rviz/slam.rviz"}.items(),
            # ),

            # SLAM
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(pkg_turtlebot3_slam, "launch", "online_async_launch.py")
            #     ),
            #     launch_arguments={"use_sim_time": use_sim_time}.items(),
            # ),

            # RViz 2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", "/home/mayooran/Documents/DRL-Robot-Navigation-ROS2/src/drl_exploration/turtlebot3_simulations/turtlebot3_gazebo/rviz/rviz.rviz",'--ros-args', '--log-level', 'fatal'],
                parameters=[{"use_sim_time": use_sim_time}],
            ),

        ]
    )
