# [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)
Humanoid walking controller with various baseline methods

[![CI](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/BaselineWalkingController/)

## Quick trial on Docker
[Install Docker](https://docs.docker.com/engine/install/ubuntu/) and execute the following commands.
```bash
$ docker pull ghcr.io/isri-aist/baseline_walking_controller:latest
$ xhost +local:
$ docker run --gpus all --rm -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ghcr.io/isri-aist/baseline_walking_controller:latest ./run_simulation.bash
```
The window of the dynamics simulator Choreonoid will open and the robot will walk.
Close the Choreonoid window to exit.

## Install

### Requirements
- Compiler supporting C++17
- Tested with Ubuntu 18.04 / ROS Melodic

### Dependencies
This package depends on
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
- [NMPC](https://github.com/isri-aist/NMPC)
- [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)

### Preparation
1. Install ROS. See [here](http://wiki.ros.org/ROS/Installation) for details.
```bash
$ export ROS_DISTRO=melodic
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-${ROS_DISTRO}-ros-base python-catkin-tools python-rosdep
```

2. Install mc_rtc. See [here](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details.
```bash
$ curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
$ sudo apt-get install libmc-rtc-dev mc-rtc-utils ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-rviz-panel libeigen-qld-dev
```

### Controller installation
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

4. Setup controller
```bash
$ mkdir -p ~/.config/mc_rtc/controllers
$ cp ~/ros/ws_bwc/src/isri-aist/BaselineWalkingController/etc/mc_rtc.yaml ~/.config/mc_rtc/mc_rtc.yaml
```

### Simulator installation
```bash
$ sudo apt-get install mc-openrtm jvrc-choreonoid
```

### Simulation execution
```bash
# Terminal 1
$ source ~/ros/ws_bwc/devel/setup.bash
$ roscore
# Terminal 2
$ source ~/ros/ws_bwc/devel/setup.bash
$ cd /usr/share/hrpsys/samples/JVRC1
$ ./clear-omninames.sh
$ choreonoid sim_mc.cnoid --start-simulation
# Terminal 3
$ source ~/ros/ws_bwc/devel/setup.bash
$ roslaunch baseline_walking_controller display.launch
```
