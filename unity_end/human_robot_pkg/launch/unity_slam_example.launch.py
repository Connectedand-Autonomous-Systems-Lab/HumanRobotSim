import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
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
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', 'src/DRL-exploration/unity_end/human_robot_pkg/rviz/human_robot.rviz', '--ros-args', '--log-level', 'fatal'],
        parameters=[{'use_sim_time':True}]
    )

    frontier_pub = Node(
        package='human_robot_pkg',
        executable='frontier_pub',
        output='screen',
        parameters=[{'use_sim_time':True}]
    )

    # ros2 run rviz2 rviz2 -d src/DRL-exploration/unity_end/human_robot_pkg/rviz/human_robot.rviz

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    cartographer_tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 'cartographer_config_dir': '/home/mayooran/Documents/DRL-Robot-Navigation-ROS2/src/drl_exploration/unity_end/human_robot_pkg/config'
            # 'slam_params_file': '/home/mayooran/Documents/human_robot_exploration_ws/src/human_robot_pkg/config/tb3_0.yaml',
            'namespace':'tb3_0'
        }.items()
    )

    slam_toolbox_human = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace':'human'
        }.items()
    )

    slam_toolbox_tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace':'tb3_0'
        }.items()
    )

    human_map_to_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'human/map']
        )

    tb3_0_map_to_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'tb3_0/map']
        )

    map_merge = Node(
            package='human_robot_pkg',
            executable='map_merge_node'
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
        # frontier_pub,
        # slam_toolbox_tb3_0,
        # slam_toolbox_human,
        # map_merge,
        # human_map_to_map,
        # tb3_0_map_to_map
    })
