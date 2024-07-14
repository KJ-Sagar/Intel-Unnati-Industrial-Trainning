# Intel-Unnati-Industrial-Trainning
---

# Creating 2D Occupancy Grid Map Using Overhead Infrastructure Cameras

This project demonstrates the creation of a 2D occupancy grid map using overhead infrastructure cameras for autonomous mobile robot (AMR) navigation. The solution leverages ROS2 and Gazebo for simulation and testing.

## Table of Contents
- [Introduction](#introduction)
- [Project Scope & Deliverables](#project-scope--deliverables)
- [Setup Instructions](#setup-instructions)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Simulation Setup](#simulation-setup)
  - [Running the Simulation](#running-the-simulation)
- [Usage](#usage)
- [Evaluation](#evaluation)
- [References](#references)

## Introduction

Autonomous mobile robots (AMRs) typically use on-board sensors for Simultaneous Localization and Mapping (SLAM) to navigate environments. This project aims to overcome the limitations of traditional SLAM by using a network of RGB cameras mounted in the environment. The goal is to create a composite 2D occupancy grid map that is accurate and can be used for navigation and path planning.

## Project Scope & Deliverables

### Project Scope
- Add 4 RGB cameras in the Gazebo simulation environment arranged in a 2x2 matrix.
- Acquire images from the simulated cameras with a resolution of 640x480 pixels.
- Develop techniques for multi-camera calibration to align the fields of view of the cameras.
- Create a composite 2D occupancy grid map with accurate physical dimensions.
- Benchmark the accuracy and computing latency of the generated map.

### Expected Deliverables
- A detailed document describing the solution, approach, novelty, pros/cons, and comparison to the state-of-the-art.
- A fused map of the environment with detailed dimensions and error estimates.
- Source code and algorithms for evaluation.

## Setup Instructions

### Prerequisites
- Ubuntu 20.04
- ROS2 Foxy
- Gazebo
- Python 3.8 or higher

### Installation

1. Install ROS2 Foxy and necessary packages:
   ```bash
   sudo apt update
   sudo apt install -y ros-foxy-desktop
   sudo apt install -y ros-foxy-turtlebot3 ros-foxy-turtlebot3-simulations
   ```

2. Set up the ROS2 workspace:
   ```bash
   mkdir -p ~/turtlebot3_ws/src
   cd ~/turtlebot3_ws
   colcon build
   source install/setup.bash
   ```

### Simulation Setup

1. Download and extract the `infraCam.zip` file:
   ```bash
   cd ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
   unzip /path/to/infraCam.zip
   ```

2. Replace the `waffle_pi.model` file:
   ```bash
   cp /path/to/waffle_pi.model ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_houses/
   ```

3. Build the workspace:
   ```bash
   cd ~/turtlebot3_ws
   colcon build
   source install/setup.bash
   ```

### Running the Simulation

1. Launch the Gazebo simulation with the overhead cameras:
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
   ```

2. Verify that the overhead camera topics are active:
   ```bash
   ros2 topic list
   ```

## Usage

- The images from the cameras are published on specific ROS topics. Use a Python script to subscribe to these topics and process the images to create a 2D occupancy grid map.

- Ensure to source the ROS2 setup script before running your Python script:
  ```bash
  source /opt/ros/foxy/setup.bash
  ```

## Evaluation

- Use Rviz to visualize and evaluate the generated map:
  ```bash
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml
  ```

- Measure the accuracy and computational latency of the map generation algorithm. Ensure the average error is less than 10% and the computational latency is under 1000ms.

## References

- [TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [ROS2 Foxy Installation](https://docs.ros.org/en/foxy/Installation.html)
- [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

For further details, refer to the provided project PDF and the TurtleBot3 manual.

---
