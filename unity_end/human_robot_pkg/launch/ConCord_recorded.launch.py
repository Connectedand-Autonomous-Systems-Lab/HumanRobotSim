import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'human_robot_pkg'
    package_dir = get_package_share_directory(package_name)
    bringup_dir = get_package_share_directory('nav2_bringup')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', 'src/unity_end/human_robot_pkg/rviz/human_robot.rviz', '--ros-args', '--log-level', 'fatal'],
        parameters=[{'use_sim_time':True}]
    )
    
    human_bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'src/unity_end/human_robot_pkg/rosbag/Concord_human_robot', ],
            output='screen'
        )
    
    return LaunchDescription({
        rviz2,
        human_bag,
    })
