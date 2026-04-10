# **ConCord: Human-in-the-Loop, Cooperative Robot Exploration**

This is the code base for implementing ROS environment block of **ConCord: Human-in-the-Loop, Cooperative Robot Exploration**.

<img src="sim-engine.jpg" width="50%" />

This simulation is intent to run along with Unity. First setup Unity Scene block mentioned in the diagram referring to the repository [Unity repository](https://github.com/Connected-and-Autonomous-Systems-Lab/Collaboration.git). 

<img src="collaborative_search_10min_16x_masked.gif" width="80%" />

## Requirements

1. ROS2 Humble [Get Started](https://docs.ros.org/en/humble/Installation.html)
2. Install following ROS2 packages 

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3-cartographer \
  ros-humble-ros-tcp-endpoint \
  ros-humble-rviz2 \
  ros-humble-tf2-ros
```
3. Clone the repository and build the ros workspace using:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Connectedand-Autonomous-Systems-Lab/HumanRobotSim.git
cd ..
colcon build --symlink-install
```

## Run Modes

Use the table below to refer the kind of experiment you want to run.

| Mode | Human | Robot | Unity Needed |
| --- | --- | --- | --- |
| [1. Visualise a complete demonstration saved in a ros bag](#1-visualise-a-complete-demonstration-saved-in-a-ros-bag) | Ros bag | Ros bag | No |
| [2. Run a robot asynchronously with a saved human bag](#2-run-a-robot-asynchronously-with-a-saved-human-bag) | Ros bag | Real time | Yes |
| [3. Run both human and robot real time](#3-run-both-human-and-robot-real-time) | Real time | Real time | Yes |

## 1. Visualise a complete demonstration saved in a ros bag 

The following command brings up an instance from the dataset and how uncoordinated exploration worked on a simulated robot in Unity along with a human.

```bash
cd ~/ros_ws
source install/setup.bash
ros2 launch human_robot_pkg uncoordinated_recorded.launch.py
```

The following command brings up how robot explored with ConCord with the same human run.
```bash
cd ~/ros_ws
source install/setup.bash
ros2 launch human_robot_pkg ConCord_recorded.launch.py
```

Run the following to compare the results of each run.
```bash
cd ~/ros_ws/src/unity_end/human_robot_pkg/results
python plot_comparisons.py
```

## 2. Run a robot asynchronously with a saved human bag

The following will run an instance from the dataset along with a robot in Unity. A sample run of the dataset is included in the dataset folder [sample run](sample_run/rosbag2_2025_12_04-17_01_38_0.db3). Change the path of the folder if necessary in line 202 of [launch file](unity_end/human_robot_pkg/launch/ConCord.launch.py). Refer the following link for the [Dataset Documentation](dataset_README.md)

1. Start the Unity and play HardScene.scene. [Unity repository](https://github.com/Connected-and-Autonomous-Systems-Lab/Collaboration.git)


2. Open three terminals and run the following.

```bash
cd ~/ros_ws
source install/setup.bash
```

3. Terminal 1: Run the launch file to start uncoordinated explo-
ration.

```bash
ros2 launch human_robot_pkg ConCord.launch.py
```

4. Terminal 2: Run nav2 launch file.

```bash
ros2 launch human_robot_pkg navigation_highlevel.launch.py
```

5. Terminal 3: Run the uncoordinated exploration.

```bash
ros2 run human_robot_pkg frontier_navigator
```

6. Visualize the RViz2 window while the robot runs in Unity. The log file for the result comparison will be saved. We suggest running both experiments for 10 minutes.

<img src="Simple_robot_on_unity_timelapse.gif" width="60%" />

## 3. Run both human and robot real time

1. In Unity, enable the human game object from the hierarchy. Play the scene.

2. Open three terminals and run the following.

```bash
cd ~/ros_ws
source install/setup.bash
```
3. Terminal 1: Run the launch file to start uncoordinated exploration.
```bash
ros2 launch human_robot_pkg real_time.launch.py
```
4. Terminal 2: Run nav2 launch file.

```bash
ros2 launch human_robot_pkg navigation_highlevel.launch.py
```

5. Terminal 3: Run the uncoordinated exploration.

```bash
ros2 run human_robot_pkg frontier_navigator
```

6.  Now in Unity, use WASD keys to explore the maze to search for victims. Use F11 to make the game view to full screen. Press C once you identify a victim and remove him from view.

7. Visualize the real-time system on RViz2. The robot will explore complementing human search.

## Known errors

### 1. Lookup error
The following error in Rviz
```bash
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

Solution:
```bash
unset GTK_PATH
```

### 2. C extension not present

```bash
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-38-x86_64-linux-gnu.so' isn't present on the system. Please refer to 'https://docs.ros.org/en/humble/Guides/Installation-Troubleshooting.html#import-failing-without-library-present-on-the-system' for possible solutions
```

Solution:
Deactivate the conda environments(deactivate even base) to go to the python version you initially installed ROS2 on. Run the python file then


### 3. Transform data too old in Controller server

```bash
[controller_server-1] [ERROR] [1775093646.921101943] [tf_help]: Transform data too old when converting from merged_map to odom
[controller_server-1] [ERROR] [1775093646.921150245] [tf_help]: Data time: 1775093630s 747304384ns, Transform time: 65s 606661987ns
[controller_server-1] [ERROR] [1775093646.967835670] [controller_server]: Failed to make progress
[controller_server-1] [WARN] [1775093646.967952823] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[bt_navigator-5] [WARN] [1775093646.996778421] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
[bt_navigator-5] [ERROR] [1775093646.996926037] [bt_navigator]: Goal failed
```

Some node is publishing old time. Most probably system time instead of ROS sim time.
