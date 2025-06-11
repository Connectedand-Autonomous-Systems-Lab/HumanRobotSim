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

# RL training
