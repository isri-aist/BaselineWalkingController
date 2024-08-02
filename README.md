# [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)
Humanoid walking controller with various baseline methods

[![CI](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/BaselineWalkingController/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/BaselineWalkingController)](https://github.com/isri-aist/BaselineWalkingController/blob/master/LICENSE)
[![Docker](https://img.shields.io/badge/Docker%20image-ready-blue)](https://github.com/isri-aist/BaselineWalkingController/pkgs/container/baseline_walking_controller)

https://user-images.githubusercontent.com/6636600/201510077-5be6ab58-9671-413a-93c4-9b84caf9735e.mp4

https://user-images.githubusercontent.com/6636600/184788681-10219f2e-cbe9-4e52-a7f7-ec3b0e1d7515.mp4

https://user-images.githubusercontent.com/6636600/184788709-fcb55fa8-fd93-4be3-ba93-d76be6fafe6a.mp4

## Features
- Completely open source! (controller framework: mc_rtc, simulator: Choreonoid, sample robot model: JVRC1)
- Full capabilities, including 3D walking, mass property error compensation of robot model, and integration with a footstep planner.
- Easy to switch between various methods of centroidal trajectory generation for walking implemented in [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection).
- Easy to switch between the two frameworks for centroidal trajectory generation for walking: (1) closed-loop MPC and (2) open-loop MPC + stabilizer.
- Support for a virtual robot whose model is publicly available so you can try out the controller right away.
- Automated management with CI: Dynamics simulation is run on CI to verify robot walking, and a Docker image is [released here](https://github.com/isri-aist/BaselineWalkingController/pkgs/container/baseline_walking_controller) with the latest version of the controller ready to run.

## Quick trial on Docker
1. (Skip if Docker is already installed.) Install Docker. See [here](https://docs.docker.com/engine/install) for details.
```bash
$ sudo apt-get install ca-certificates curl gnupg lsb-release
$ sudo mkdir -p /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

To enable GPUs in Docker (i.e., enable the `--gpus` option in the `docker` command), install `nvidia-docker2`.
See [here](https://www.ibm.com/docs/en/maximo-vi/continuous-delivery?topic=planning-installing-docker-nvidia-docker2) for details.

2. By executing the following commands, the window of the dynamics simulator Choreonoid will open and the robot will walk.
Close the Choreonoid window to exit.
```bash
$ docker pull ghcr.io/isri-aist/baseline_walking_controller:latest
$ xhost +local:
```

- Simulate walking on a plain like in [this video](https://user-images.githubusercontent.com/6636600/184788681-10219f2e-cbe9-4e52-a7f7-ec3b0e1d7515.mp4).
```bash
$ docker run --gpus all --rm -it \
  --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ghcr.io/isri-aist/baseline_walking_controller:latest ./walk_on_plane.bash
```

- Simulate walking on stairs like in [this video](https://user-images.githubusercontent.com/6636600/184788709-fcb55fa8-fd93-4be3-ba93-d76be6fafe6a.mp4).
```bash
$ docker run --gpus all --rm -it \
  --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ghcr.io/isri-aist/baseline_walking_controller:latest ./walk_on_stairs.bash
```

- Simulate walking with planning footstep sequence avoiding obstacles like in [this video](https://user-images.githubusercontent.com/6636600/187928560-9835e148-bd4b-4b12-8c65-ead6cf3d92f1.mp4).
```bash
$ docker run --gpus all --rm -it \
  --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ghcr.io/isri-aist/baseline_walking_controller:latest ./walk_with_footstep_planner.bash
```

The Docker image is automatically updated on [CI](https://github.com/isri-aist/BaselineWalkingController/actions/workflows/docker.yaml) from [this Dockerfile](https://github.com/isri-aist/BaselineWalkingController/blob/master/.github/workflows/Dockerfile).

## Technical details
This controller is a simple combination of the following existing typical elemental methods in the field of biped robotics:
- CoM trajectory generation based on LIPM
- ZMP feedback based on DCM
- Wrench distribution
- Foot damping control

For more information on the technical details, please see the following papers:
- Papers listed in [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- Papers listed in [ForceControlCollection](https://github.com/isri-aist/ForceControlCollection)
- S Kajita, et al. Biped walking stabilization based on linear inverted pendulum tracking. IROS, 2010.
- S Caron, et al. Stair climbing stabilization of the HRP-4 humanoid robot using whole-body admittance control. ICRA, 2019.

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)

This package also depends on the following packages. However, manual installation is unnecessary when this package is installed using `wstool` as described in [Controller installation](#controller-installation).
- [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
- [ForceControlCollection](https://github.com/isri-aist/ForceControlCollection)
- [TrajectoryCollection](https://github.com/isri-aist/TrajectoryCollection)
- [NMPC](https://github.com/isri-aist/NMPC)
- [CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- [BaselineFootstepPlanner](https://github.com/isri-aist/BaselineFootstepPlanner) (used only when found)

### Preparation
1. (Skip if ROS is already installed.) Install ROS. See [here](http://wiki.ros.org/ROS/Installation) for details.
```bash
$ export ROS_DISTRO=melodic
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-${ROS_DISTRO}-ros-base python-catkin-tools python-rosdep
```

2. (Skip if mc_rtc is already installed.) Install mc_rtc. See [here](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details.
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
$ wstool set -t src isri-aist/BaselineWalkingController https://github.com/isri-aist/BaselineWalkingController --git -y
$ wstool update -t src isri-aist/BaselineWalkingController
$ wstool merge -t src src/isri-aist/BaselineWalkingController/depends.rosinstall
$ wstool update -t src
```

2. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

3. Build a package.
```bash
$ catkin build baseline_walking_controller -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_QLD=ON --catkin-make-args all tests
```

4. Setup controller
```bash
$ mkdir -p ~/.config/mc_rtc/controllers
$ cp ~/ros/ws_bwc/src/isri-aist/BaselineWalkingController/etc/mc_rtc.yaml ~/.config/mc_rtc/mc_rtc.yaml
```

5. Setup motion configuration file (optional)
```bash
$ roscd baseline_walking_controller
$ cd .github/workflows
$ python ./scripts/mergeConfigs.py ./config/PreviewControlZmp.yaml ./config/OpenLoopMpc.yaml ./config/WalkingOnPlane.yaml > ~/.config/mc_rtc/controllers/BaselineWalkingController.yaml
```

### Simulator installation
```bash
$ sudo apt-get install mc-state-observation jvrc-choreonoid
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

## Documents
[BaselineWalkingController Tips](https://www.dropbox.com/scl/fi/vpn4hvbh1v037aubm4jjo/BaselineWalkingController-Tips.docx?rlkey=4t7fvb45tmvmlb23awb3ke5eo&st=fryrxj6w&dl=0): a collection of tips for BaselineWalkingController users

## Controllers for motions beyond walking
The following controllers are based on or developed with the same philosophy as BaselineWalkingController.
- Loco-manipulation: [LocomanipController](https://github.com/isri-aist/LocomanipController)
- Multi-contact motion: [MultiContactController](https://github.com/isri-aist/MultiContactController)
