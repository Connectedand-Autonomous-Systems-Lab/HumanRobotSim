from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    namespace = LaunchConfiguration('namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=['src/DRL-exploration/unity_end/human_robot_pkg/config/' , LaunchConfiguration('namespace'), '.yaml'],
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time},
          {'odom_frame': [LaunchConfiguration('namespace'), '/odom' ]},
          {'base_frame': [LaunchConfiguration('namespace'), '/base_footprint']},
          {'map_frame': [LaunchConfiguration('namespace'), '/map']},
          {'scan_topic': ['/', LaunchConfiguration('namespace'), '/scan']},
          {'max_laser_range': 50.0},
          {'min_laser_range': 0.0},
          {'map_update_interval': 0.05},
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
                ('/scan', ['/' , LaunchConfiguration('namespace'),'/scan']),
                ('/map', [ '/' ,LaunchConfiguration('namespace'), '/map']),
            ],
        )
    
    group = GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            start_async_slam_toolbox_node
        ])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(group)

    return ld