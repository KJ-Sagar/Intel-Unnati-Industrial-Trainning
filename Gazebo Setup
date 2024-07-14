# Gazebo Simulation Setup

This guide will help you set up the Gazebo simulation environment for TurtleBot3.

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Gazebo

## Installation

1. **Install ROS2 Foxy:**
    ```bash
    sudo apt update
    sudo apt install -y ros-foxy-desktop
    ```

2. **Install Gazebo:**
    ```bash
    sudo apt install -y gazebo11
    sudo apt install -y ros-foxy-gazebo-ros-pkgs
    ```

3. **Install TurtleBot3 Packages:**
    ```bash
    sudo apt install -y ros-foxy-turtlebot3 ros-foxy-turtlebot3-simulations
    ```

4. **Set TurtleBot3 Model:**
    ```bash
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
    source ~/.bashrc
    ```

## Running the Simulation

1. **Launch the Gazebo simulation:**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. **Control the TurtleBot3:**
    Use teleop commands to control the robot.
    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

## Resources

- [TurtleBot3 Gazebo Simulation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

For any issues, refer to the [ROS2 and Gazebo troubleshooting guide](https://answers.ros.org/questions/).
