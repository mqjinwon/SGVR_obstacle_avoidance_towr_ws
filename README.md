# Project Title

Obstacle avoidance for a legged robot

## Description

This repository about collision avoidance planner for a legged robot

## Getting Started

### Dependencies

-   Ubuntu 18.04
-   ROS Melodic

### Installing

-   Install prerequisites

```
sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
sudo apt-get install ros-melodic-grid-map*
sudo apt-get install libncurses5-dev libncursesw5-dev
```

### Build
```
catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Executing program

```
roslaunch towr_ros towr_ros.launch # execute program
rosrun towr_ros towr_user_interface # execute user interface
```

## Help

Any advise for common problems or issues.

## Authors

[Jinwom Kim](https://github.com/mqjinwon) \
Heechan Shin \
Sung-Eui Yoon

## Conference Paper

[Collision-Backpropagation based Obstacle Avoidance Methodfor a Legged Robot Expressed as a Simplified Dynamics Model](https://drive.google.com/file/d/1J9qv8Xt46Gzt9eYBwg6-yQvde4DOqUVs/view)
