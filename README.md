# My Bot ROS2 Simulation

This repository contains a ROS2 package for simulating an articulated robot
in Gazebo.

## Features
- URDF/XACRO robot description
- Gazebo simulation
- ROS2 control integration
- Custom worlds for testing

## Tools Used
- ROS2
- Gazebo
- URDF / XACRO
- Python

## How to Run
```bash
colcon build
source install/setup.bash
ros2 launch launch_sim.launch.py
