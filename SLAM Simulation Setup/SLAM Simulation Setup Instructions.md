# SLAM Simulation Setup

This guide will help you set up the SLAM simulation environment for TurtleBot3.

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Gazebo
- TurtleBot3 Packages

## Installation

Follow the installation steps from the [Gazebo Simulation Setup](../gazebo_simulation/README.md) to install ROS2, Gazebo, and TurtleBot3 packages.

## Running the SLAM Simulation

1. **Launch the SLAM simulation:**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
    ```

2. **Start SLAM Node:**
    ```bash
    ros2 launch turtlebot3_slam turtlebot3_slam.launch.py
    ```

3. **Control the TurtleBot3:**
    Use teleop commands to control the robot.
    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

4. **Save the Map:**
    Once you have explored the environment, save the map.
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/map
    ```

## Resources

- [TurtleBot3 SLAM Simulation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/)

For any issues, refer to the [ROS2 and Gazebo troubleshooting guide](https://answers.ros.org/questions/).
