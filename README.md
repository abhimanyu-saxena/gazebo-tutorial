# gazebo-tutorials

## Overview

This package runs a turtlebot3 (Wafflepi) in a gazebo world and implements the roomba collision avoidance algorithim on it.

## Building and Running

Clone the repository in your ROS2 workspace.

```sh
cd < path_to_your_workspace >/src

# For cloning using SSH
git clone https://github.com/abhimanyu-saxena/gazebo-tutorial.git
```

### Building

To build the package follow the following steps

```sh
cd .. # Make sure you are in the ros2 workspace folder and not in src

# To build the package use
colcon build --package-select gazebo-tutorial

# Source your setup file
source install/setup.bash
```

### Running

```sh
cd < path_to_your_workspace >/src/gazebo-tutorial

export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/

export TURTLEBOT3_MODEL=waffle_pi

# Using ROS2 Launch
ros2 launch gazebo-tutorial launch_roomba.py record:=true
```

## Cppcheck and Cpplint

To run the CppCheck

```sh
sh cppcheck.sh
```

To run the Cpplint

```sh
sh cpplint.sh
```

## Results

Results for `cppcheck`, `cpplint` can be viewed in `results` folder

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- CMake Version 3.8 or greater
- C++ 17 or newer
- [Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- Turtlebot3
- Turtlebot3 Gazebo

## Assumptions

The above instruction assume that you have installed all the Dependecies and are working on a Ubuntu 22.04 LTS system and have created your ROS2 workspace beforehand.