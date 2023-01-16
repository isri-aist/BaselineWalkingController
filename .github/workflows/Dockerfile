FROM ubuntu:20.04
MAINTAINER m-murooka@aist.go.jp

ARG REPOSITORY=isri-aist
ARG COMMIT_SHA=master
ENV REPOSITORY ${REPOSITORY}
ENV COMMIT_SHA ${COMMIT_SHA}

# https://qiita.com/haessal/items/0a83fe9fa1ac00ed5ee9
ENV DEBCONF_NOWARNINGS=yes
# https://qiita.com/yagince/items/deba267f789604643bab
ENV DEBIAN_FRONTEND=noninteractive
# https://qiita.com/jacob_327/items/e99ca1cf8167d4c1486d
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

# https://stackoverflow.com/a/25423366
SHELL ["/bin/bash", "-c"]

# Install basic packages
RUN apt-get update -qq
RUN apt-get install -y sudo aptitude build-essential lsb-release wget gnupg2 curl emacs
RUN aptitude update -q

# Install ROS
ENV ROS_DISTRO noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get update -qq
RUN apt-get install -y ros-${ROS_DISTRO}-ros-base python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall python3-rosinstall-generator doxygen graphviz

# Install mc_rtc
RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash
RUN apt-get install -y libmc-rtc-dev mc-rtc-utils ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-rviz-panel libeigen-qld-dev
# https://github.com/jrl-umi3218/lipm_walking_controller/blob/b564d655388ae6a6725c504e5c74a62192e58c7c/.github/workflows/build.yml#L69
RUN apt-get install -y mc-state-observation jvrc-choreonoid xvfb ffmpeg mesa-utils fluxbox xserver-xorg xserver-xorg-core xserver-xorg-video-all libwayland-egl1-mesa

# Enable nvidia
# https://qiita.com/eisoku9618/items/c2cca0f0bf764def2efd
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Setup catkin workspace
ENV HOME /root
RUN mkdir -p ${HOME}/catkin_ws/src
WORKDIR ${HOME}/catkin_ws
RUN wstool init src
# To handle CI triggered by Pull Request, COMMIT_SHA must be specified in the git fetch argument,
# but since wstool does not support it, execute the git commands directly instead.
# RUN wstool set -t src BaselineWalkingController https://github.com/${REPOSITORY}/BaselineWalkingController --git -y -v ${COMMIT_SHA}
RUN mkdir -p src/isri-aist/BaselineWalkingController && \
    cd src/isri-aist/BaselineWalkingController && \
    git init && \
    git remote add origin https://github.com/${REPOSITORY}/BaselineWalkingController && \
    git fetch origin ${COMMIT_SHA} && \
    git checkout ${COMMIT_SHA} && \
    git submodule update --init --recursive
RUN wstool merge -t src src/isri-aist/BaselineWalkingController/depends.rosinstall
RUN wstool update -t src

# Rosdep install
RUN rosdep init
RUN rosdep update
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install -y -r --from-paths src --ignore-src

# Catkin build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build baseline_footstep_planner -DCMAKE_BUILD_TYPE=RelWithDebInfo
RUN source ${HOME}/catkin_ws/devel/setup.bash && catkin build baseline_walking_controller -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_QLD=ON
RUN echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc

# Setup simulation
WORKDIR ${HOME}
RUN mkdir -p ${HOME}/.config/mc_rtc/controllers
RUN cp ${HOME}/catkin_ws/src/isri-aist/BaselineWalkingController/etc/mc_rtc.yaml ${HOME}/.config/mc_rtc/mc_rtc.yaml
RUN echo $'#!/usr/bin/env bash \n\
source ${HOME}/catkin_ws/devel/setup.bash \n\
cp ${HOME}/catkin_ws/src/isri-aist/BaselineWalkingController/.github/workflows/config/WalkingOnPlane.yaml ${HOME}/.config/mc_rtc/controllers/BaselineWalkingController.yaml \n\
/usr/share/hrpsys/samples/JVRC1/clear-omninames.sh \n\
cd /usr/share/hrpsys/samples/JVRC1 && choreonoid --start-simulation sim_mc.cnoid' > ${HOME}/walk_on_plane.bash
RUN echo $'#!/usr/bin/env bash \n\
source ${HOME}/catkin_ws/devel/setup.bash \n\
cp ${HOME}/catkin_ws/src/isri-aist/BaselineWalkingController/.github/workflows/config/WalkingOnStairs.yaml ${HOME}/.config/mc_rtc/controllers/BaselineWalkingController.yaml \n\
/usr/share/hrpsys/samples/JVRC1/clear-omninames.sh \n\
cd /usr/share/hrpsys/samples/JVRC1 && choreonoid --start-simulation sim_mc_comanoid_staircase.cnoid' > ${HOME}/walk_on_stairs.bash
RUN echo $'#!/usr/bin/env bash \n\
source ${HOME}/catkin_ws/devel/setup.bash \n\
cp ${HOME}/catkin_ws/src/isri-aist/BaselineWalkingController/.github/workflows/config/WalkingWithFootstepPlanner.yaml ${HOME}/.config/mc_rtc/controllers/BaselineWalkingController.yaml \n\
/usr/share/hrpsys/samples/JVRC1/clear-omninames.sh \n\
roscore > /dev/null 2>&1 < /dev/null & \n\
ROSCORE_PID=$! \n\
sleep 1s \n\
roslaunch baseline_walking_controller display.launch > /dev/null 2>&1 < /dev/null & \n\
RVIZ_PID=$! \n\
sleep 1s \n\
cd /usr/share/hrpsys/samples/JVRC1 && choreonoid --start-simulation sim_mc_comanoid_staircase.cnoid \n\
kill -2 ${ROSCORE_PID} \n\
kill -2 ${RVIZ_PID}' > ${HOME}/walk_with_footstep_planner.bash
RUN chmod 755 ${HOME}/walk_on_plane.bash
RUN chmod 755 ${HOME}/walk_on_stairs.bash
RUN chmod 755 ${HOME}/walk_with_footstep_planner.bash
