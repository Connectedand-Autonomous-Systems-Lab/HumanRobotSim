# Dataset Overview

This dataset includes multimodal recordings from a human-robot interaction session. It contains gaze data, perspective view images, LiDAR scans, and map data for further analysis and visualization.

## Contents

### 📁 `gaze_log.csv`
- Timestamped gaze points from the human player.
- Captured using the Beam Eye Tracker via webcam.
- Note: Gaze data may contain inaccuracies or biases due to webcam-based capture.

### 📁 `pv/`
- Timestamped perspective view images from the human player's point of view.
- A projection script is available in the `scripts/` folder to overlay gaze data onto these images.

### 📁 `rosbag/`
Contains recorded ROS 2 bag files with the following topics:
- Human LiDAR scans
- Robot LiDAR scans
- Transformations (`/tf`)
- Human gaze data

### 📁 `costmap/`
- Contains the merged map generated from both human and robot perspectives.

### 📁 `scripts/`
- Includes a Python script to project gaze points onto PV images using timestamp alignment.

---
