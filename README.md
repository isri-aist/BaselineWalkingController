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

### Installation 

#### With mc-rtc-superbuild 

#### Building with mc-rtc-superbuild (recommanded)

The recommended way to build this project is to use mc-rtc-superbuild along with the extension for BaselineWalkingController controller provided in superbuild-extensions.

```bash
git clone --recursive https://github.com/mc-rtc/mc-rtc-superbuild.git
cd extensions
git clone --recursive https://github.com/mc-rtc/superbuild-extensions.git
echo "set(EXTENSIONS_DIR ${CMAKE_CURRENT_LIST_DIR}/superbuild-extensions)" > local.cmake
# Install baseline walking controller and its dependencies
echo "include(${EXTENSIONS_DIR}/controllers/BaseLineWalkingController.cmake)" >> local.cmake
# For dynamics simulation with MuJoCo
echo "include(${EXTENSIONS_DIR}/simulation/MuJoCo.cmake)" >> local.cmake
echo "include(${EXTENSIONS_DIR}/gui/mc_rtc-magnum.cmake)" >> local.cmake
cd ..
# Configure the superbuild and install all required system dependencies
cmake --preset relwithdebinfo
# Clone all projects and their dependencies, and build them
# Note by default this will create a workspace folder in the parent directory
# If you wish to change the path or default options, edit CMakePresets.json or create your own preset in CMakeUserPresets.json
cmake --build --preset relwithdebinfo
```
### Setup simulation

1. Setup controller
```bash
$ mkdir -p ~/.config/mc_rtc/controllers
$ cp ~/workspace/install/etc/mc_rtc.yaml ~/.config/mc_rtc/mc_rtc.yaml
```
Edit `Enabled` in `~/.config/mc_rtc/mc_rtc.yaml` to be `Enabled: BaselineWalkingController`.

2. Setup motion configuration file (optional)
```bash
$ cd ~/workspace/devel/BaseLineWalkingController/.github/workflows
$ python ./scripts/mergeConfigs.py ./config/PreviewControlZmp.yaml ./config/OpenLoopMpc.yaml ./config/WalkingOnPlane.yaml > ~/.config/mc_rtc/controllers/BaselineWalkingController.yaml
```

### Simulation execution
```bash
$ mc_mujoco --sync
```

## Documents
[BaselineWalkingController Tips](https://www.dropbox.com/scl/fi/vpn4hvbh1v037aubm4jjo/BaselineWalkingController-Tips.docx?rlkey=4t7fvb45tmvmlb23awb3ke5eo&st=fryrxj6w&dl=0): a collection of tips for BaselineWalkingController users

## Controllers for motions beyond walking
The following controllers are based on or developed with the same philosophy as BaselineWalkingController.
- Loco-manipulation: [LocomanipController](https://github.com/isri-aist/LocomanipController)
- Multi-contact motion: [MultiContactController](https://github.com/isri-aist/MultiContactController)

