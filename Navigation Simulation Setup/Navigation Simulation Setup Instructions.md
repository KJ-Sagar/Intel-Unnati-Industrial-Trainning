# Navigation Simulation Setup

This guide will help you set up the navigation simulation environment for TurtleBot3.

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Gazebo
- TurtleBot3 Packages

## Installation

Follow the installation steps from the [Gazebo Simulation Setup](../gazebo_simulation/README.md) to install ROS2, Gazebo, and TurtleBot3 packages.

## Running the Navigation Simulation

1. **Launch the Navigation simulation:**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. **Start Navigation Node:**
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<path_to_map.yaml>
    ```

3. **Send Navigation Goals:**
    Use RViz to set navigation goals and visualize the robot's path.

## Resources

- [TurtleBot3 Navigation Simulation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/)

For any issues, refer to the [ROS2 and Gazebo troubleshooting guide](https://answers.ros.org/questions/).
