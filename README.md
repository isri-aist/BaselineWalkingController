# [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)
Humanoid walking controller with various baseline methods

[![CI](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/BaselineWalkingController/)

## Install

### Requirements
- Compiler supporting C++17
- Tested with Ubuntu 18.04 / ROS Melodic

### Dependencies
This package depends on
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
- [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)

### Installation procedure
It is assumed that [ROS](http://wiki.ros.org/ROS/Installation) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) is installed.

1. Setup catkin workspace.
```bash
$ mkdir -p ~/ros/ws_bwc/src
$ cd ~/ros/ws_bwc
$ wstool init src
$ wstool set -t src isri-aist/QpSolverCollection git@github.com:isri-aist/QpSolverCollection.git --git -y
$ wstool set -t src isri-aist/NMPC git@github.com:isri-aist/NMPC.git --git -y
$ wstool set -t src isri-aist/CentroidalControlCollection git@github.com:isri-aist/CentroidalControlCollection.git --git -y
$ wstool set -t src isri-aist/BaselineWalkingController git@github.com:isri-aist/BaselineWalkingController.git --git -y
$ wstool update -t src
```

2. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

3. Build a package.
```bash
$ catkin build baseline_walking_controller -DCMAKE_BUILD_TYPE=RelWithDebInfo --catkin-make-args all tests
```
