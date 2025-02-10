import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_name = 'human_robot_pkg'
    package_dir = get_package_share_directory(package_name)

    ros_tcp_endpoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
        ),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', '/home/mayooran/Documents/human_robot_exploration_ws/src/human_robot_pkg/rviz/human_robot.rviz'],
        parameters=[{'use_sim_time':True}]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    slam_toolbox_tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'online_async_launch.py')
            # os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 'slam_params_file': '/home/mayooran/Documents/human_robot_exploration_ws/src/human_robot_pkg/config/tb3_0.yaml',
            'namespace':'tb3_0'
        }.items()
    )

    slam_toolbox_human = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'online_async_launch.py')
            # os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': '/home/mayooran/Documents/human_robot_exploration_ws/src/human_robot_pkg/config/human.yaml',
            'namespace':'human'
        }.items()
    )

    map_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('multirobot_map_merge'), 'launch', 'map_merge.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    tb3_0_slam_toolbox = GroupAction(
        actions=[
            PushRosNamespace('tb3_0'),
            slam_toolbox_tb3_0,
        ]
    )

    tb3_0_nav2_bringup = GroupAction(
        actions=[
            PushRosNamespace('tb3_0'),
            nav2_bringup,
        ]
    )
    
    return LaunchDescription({
        ros_tcp_endpoint,
        rviz2,
        # tb3_0_nav2_bringup,
        # tb3_0_slam_toolbox
        slam_toolbox_tb3_0,
        # slam_toolbox_human,
        # map_merge
    })
