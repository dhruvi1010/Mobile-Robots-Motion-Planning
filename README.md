# Mobile-Robots-Motion-Planning
## Autonomous Robot Navigation and Path Optimization Using RRT and TEB in ROS and Gazebo

This project involves implementing an autonomous robot navigation system in a simulated environment using ROS Noetic and Gazebo. The primary focus is on path planning and optimization utilizing the Rapidly-exploring Random Tree (RRT) algorithm for global planning and the Timed Elastic Band (TEB) algorithm for local path optimization and obstacle avoidance. The implementation includes setting up ROS parameters, resetting the simulation environment, generating and visualizing occupancy maps, and executing the navigation plan while dynamically adjusting to changes in the environment. The project leverages ROS Noetic packages for seamless integration and operation.
# Autonomous Robot Navigation and Path Optimization Using RRT and TEB in ROS and Gazebo

## Overview

This project demonstrates an autonomous robot navigation system in a simulated environment using ROS Noetic and Gazebo. The system focuses on path planning and optimization using the Rapidly-exploring Random Tree (RRT) algorithm for global path planning and the Timed Elastic Band (TEB) algorithm for local path optimization and obstacle avoidance.

## Table of Contents

1. [Setup](#setup)
2. [Project Structure](#project-structure)
3. [Usage](#usage)
4. [Helper Functions](#helper-functions)
5. [References](#references)

## Setup

### Prerequisites

- ROS Noetic
- Gazebo
- MATLAB (with ROS Toolbox)

### Installation

1. Install ROS Noetic on your system. Follow the official [ROS installation guide](http://wiki.ros.org/noetic/Installation) for your OS.
2. Install Gazebo if it's not already installed. Follow the official [Gazebo installation guide](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
3. Ensure MATLAB is installed and the ROS Toolbox is available.

### ROS Packages

The project utilizes several ROS Noetic packages. Ensure you have the following packages installed:

- `roscpp`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf`
- `gazebo_ros`

## Project Structure

```
.
├── scripts
│   ├── navigation.m          # Main script for robot navigation
│   └── helper_functions.m    # Helper functions for navigation
├── maps
│   └── rst_lab_cropped.pgm   # Occupancy grid map of the environment
├── README.md                 # Project documentation
└── LICENSE                   # Project license
```

## Usage

### ROS Setup

1. Ensure the ROS master node is running:

    ```bash
    roscore
    ```

2. Launch the Gazebo simulation environment:

    ```bash
    roslaunch gazebo_ros empty_world.launch
    ```

3. Open MATLAB and navigate to the project directory.

### Running the Navigation Script

1. Set the ROS environment variables and initialize the ROS node:

    ```matlab
    setenv('ROS_MASTER_URI', 'http://192.168.**.**:11311'); % Your ROS_MASTER_URI
    setenv('ROS_IP', '192.168.**.***');  % IP of UBUNTU
    rosinit;
    ```
In case the Firewall blocks the access, allow Matlab to access the network

2. Run the main navigation script:

    ```matlab
    run('scripts/navigation.m');
    ```

3. The robot will reset its position, load the occupancy grid map, and begin executing the navigation plan using the specified global and local planners.

### Global and Local Path Planning

- The global path is planned using the RRT algorithm.
- The local path is optimized using the TEB algorithm to handle dynamic obstacles and smooth the path.

## Helper Functions

### goalCallback

Handles goal position updates from ROS.

```matlab
function [] = goalCallback(~, msg, goalHandle)
    goalHandle.x = msg.Pose.Position.X;
    goalHandle.y = msg.Pose.Position.Y;
    eul = quat2eul([msg.Pose.Orientation.W, msg.Pose.Orientation.X, ...
                    msg.Pose.Orientation.Y, msg.Pose.Orientation.Z]);
    goalHandle.theta = eul(1); % Extract yaw angle
    disp("Received goal");
end
```

### OdometryMsg2Pose

Converts odometry messages to a pose vector.

```matlab
function [ x, y, theta ] = OdometryMsg2Pose(poseMsg)
    x = poseMsg.Pose.Pose.Position.X;
    y = poseMsg.Pose.Pose.Position.Y;
    eul = quat2eul([poseMsg.Pose.Pose.Orientation.W, poseMsg.Pose.Pose.Orientation.X,  poseMsg.Pose.Pose.Orientation.Y, poseMsg.Pose.Pose.Orientation.Z]);
    theta = eul(1);
end
```

### goalHandle2goalPose

Converts goal handle to goal pose.

```matlab
function goalPose = goalHandle2goalPose(goalHandle)
    goalPose = [goalHandle.x, goalHandle.y, goalHandle.theta];
end
```

## References

- [ROS Noetic](http://wiki.ros.org/noetic)
- [Gazebo](http://gazebosim.org/)
- [MATLAB ROS Toolbox](https://www.mathworks.com/products/ros.html)
- [Rapidly-exploring Random Tree (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)
- [Timed Elastic Band (TEB)](http://wiki.ros.org/teb_local_planner)
