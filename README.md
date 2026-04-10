# ConCord Simulation Engine

ConCord is a collaborative exploration system that incorporates humans as real-time agents in the robotic ecosystem to support cooperative search and exploration. This simulation engine combines human sensor data (Point-of-View images, motion, depth scans), robot autonomy, Unity-based simulation, and ROS 2 tooling to provide a scalable experimentation in simulation.

<p align="center">
  <img src="readme_files/sim-engine.jpg" alt="ConCord simulation pipeline" width="55%" />
</p>

## Overview

This repository contains the ROS-side workspace and supporting assets for running ConCord experiments. The project is designed to work alongside the Unity simulation environment, with ROS2 integration for handling simultaneous localization and mapping, frontier exploration, navigation, and experiment orchestration.

## Setup the system in following order

- Unity setup: [Unity repository](https://github.com/Connected-and-Autonomous-Systems-Lab/Collaboration.git)
- ROS workspace setup: [ROS workspace](readme_files/rosws_README.md)
