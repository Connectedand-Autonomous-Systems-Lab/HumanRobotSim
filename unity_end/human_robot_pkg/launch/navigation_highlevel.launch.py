import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    package_name = 'human_robot_pkg'
    package_dir = get_package_share_directory(package_name)
    bringup_dir = get_package_share_directory('nav2_bringup')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
                          'use_sim_time': 'True',
                          'params_file': os.path.join(get_package_share_directory('human_robot_pkg'), 'config', 'nav2_params.yaml'),
                        #   'params_file': os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
                          }.items())
    
    # ros2 launch nav2_bringup navigation_launch.py params_file:=/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/config/nav2_params.yaml


    
    simple_navigator = Node(
        package="human_robot_pkg",
        executable="simple_navigator"
    )
    
    map_logger = Node(
        package="human_robot_pkg",
        executable="map_logger"
    )

    odom_pub = Node(
        package="human_robot_pkg",
        executable="odom_pub"
    )


    human_bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/rosbag/1'],
            output='screen'
        )

    move_simple = [
        'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/Twist',
        "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
        '-1'
    ]

    move1 = ExecuteProcess(cmd=move_simple, output='screen')
    move2 = TimerAction(period=2.0, actions=[ExecuteProcess(cmd=move_simple, output='screen')])
    move3 = TimerAction(period=4.0, actions=[ExecuteProcess(cmd=move_simple, output='screen')])

        
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            # navigation,
            move1,
            move2,
            move3,
            simple_navigator,
            map_logger,
            # human_bag
            ]
    )

    return LaunchDescription({
        navigation,
        odom_pub,
        delayed_nodes,
        # simple_navigator,
        # map_logger,
        # human_bag
    })
