# Project Documentation

This directory contains all the documentation related to the "Creating 2D Occupancy Grid Map Using Overhead Infrastructure Cameras" project. The documentation is organized to provide comprehensive information about the project's objectives, setup, implementation, and evaluation.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Setup Guides](#setup-guides)
    - [Gazebo Simulation Setup](#gazebo-simulation-setup)
    - [SLAM Simulation Setup](#slam-simulation-setup)
    - [Navigation Simulation Setup](#navigation-simulation-setup)
3. [Implementation Details](#implementation-details)
4. [Evaluation and Results](#evaluation-and-results)
5. [References](#references)

## Project Overview

The main goal of this project is to create a 2D occupancy grid map using overhead infrastructure cameras for autonomous mobile robot (AMR) navigation. The solution leverages ROS2 and Gazebo for simulation and testing, and aims to improve upon traditional SLAM methods by utilizing a network of RGB cameras mounted in the environment.

## Setup Guides

### Gazebo Simulation Setup

The Gazebo Simulation Setup guide provides step-by-step instructions to set up the Gazebo simulation environment for TurtleBot3.

[Read more](../Gazebo_Setup/Gazebo_Setup_Instructions.md)

### SLAM Simulation Setup

The SLAM Simulation Setup guide provides step-by-step instructions to set up the SLAM simulation environment for TurtleBot3.

[Read more](../SLAM_Simulation_Setup/SLAM_Simulation_Setup_Instructions.md)

### Navigation Simulation Setup

The Navigation Simulation Setup guide provides step-by-step instructions to set up the navigation simulation environment for TurtleBot3.

[Read more](../nav_simulation/README.md)

## Implementation Details

This section includes detailed documentation on the implementation of various components of the project:
- Camera calibration
- Image acquisition and processing
- Map fusion techniques

### Files:
- `camera_calibration.md`
- `image_acquisition.md`
- `map_fusion.md`

## Evaluation and Results

This section includes documentation on the evaluation methodology, benchmarking results, and analysis of the project's outcomes. It contains details on how the accuracy and computing latency of the generated map were measured and evaluated.

### Files:
- `evaluation.md`
- `results.md`

## References

A list of resources and references used throughout the project:
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/Installation.html)
- [Gazebo Simulation Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

For further information, please refer to the respective documentation files within this directory.
