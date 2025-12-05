# Run saved human rosbag along with the turtlebot3

```bash
ros2 launch human_in_the_loop human_in_the_loop.launch.py
```

Make sure you disable the human player in the Unity scene as you are just using the previous run of the human run.

```bash
ros2 launch human_in_the_loop navigation_highlevel.launch.py
```

Finally,

```bash
ros2 launch explore_lite explore.launch.py
```

If you want to test the experimental “dev” explorer (which considers human pose information when picking frontiers) build and launch `explore_dev.cpp` / `frontier_search_dev.cpp` via the `explore_dev.launch.py` file instead; otherwise the standard launch described above ignores the human pose topic entirely.

# RL training


# Known errors

## 1. Lookup error
The following error in Rviz
```bash
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

Solution:
```bash
unset GTK_PATH
```

## 2. C extension not present

```bash
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-38-x86_64-linux-gnu.so' isn't present on the system. Please refer to 'https://docs.ros.org/en/humble/Guides/Installation-Troubleshooting.html#import-failing-without-library-present-on-the-system' for possible solutions
```

Solution:
Deactivate the conda environments(deactivate even base) to go to the python version you initially installed ROS2 on. Run the python file then
