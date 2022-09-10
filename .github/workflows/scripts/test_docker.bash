#!/usr/bin/env bash

# Check repository
set -e
. ${HOME}/catkin_ws/devel/setup.bash
roscd baseline_walking_controller
set -x
git remote get-url origin
git log HEAD^..HEAD

# Run simulation
set -e
. ${HOME}/catkin_ws/devel/setup.bash
set -x
export DISPLAY=":1"
Xvfb ${DISPLAY} -screen 0 1920x1080x24 &
sleep 10s
fluxbox 2> /dev/null &
mkdir -p ${HOME}/.config/mc_rtc/controllers
cp ${HOME}/catkin_ws/src/BaselineWalkingController/etc/mc_rtc.yaml ${HOME}/.config/mc_rtc
CI_DIR=${HOME}/catkin_ws/src/BaselineWalkingController/.github/workflows
python3 ${CI_DIR}/scripts/mergeConfigs.py ${CI_DIR}/config/${MPC_METHOD}.yaml ${CI_DIR}/config/${MPC_FRAMEWORK}.yaml ${CI_DIR}/config/${MOTION_TYPE}.yaml > ${HOME}/.config/mc_rtc/controllers/BaselineWalkingController.yaml
ffmpeg -y -f x11grab -s 1920x1080 -r 30 -i ${DISPLAY} -qscale 0 -vcodec huffyuv /tmp/video.avi > /dev/null 2>&1 < /dev/null &
FFMPEG_PID=$!
cd /usr/share/hrpsys/samples/JVRC1
./clear-omninames.sh
if [ "${MOTION_TYPE}" == "WalkingOnPlane" ]; then
  CNOID_FILE=sim_mc.cnoid
else
  CNOID_FILE=sim_mc_comanoid_staircase.cnoid
fi
choreonoid --start-simulation ${CNOID_FILE} &
CNOID_PID=$!
if [ "${MOTION_TYPE}" == "WalkingWithFootstepPlanner" ]; then
  SLEEP_DURATION="60s"
else
  SLEEP_DURATION="40s"
fi
sleep ${SLEEP_DURATION}
kill -2 ${CNOID_PID}
kill -2 ${FFMPEG_PID}
sleep 10s
mkdir -p /tmp/results
ffmpeg -nostats -i /tmp/video.avi /tmp/results/BWC-video-${RESULTS_POSTFIX}.mp4
LOG_FILENAME=BWC-log-${RESULTS_POSTFIX}
mv `readlink -f /tmp/mc-control-BaselineWalkingController-latest.bin` /tmp/${LOG_FILENAME}.bin
tar czf /tmp/results/${LOG_FILENAME}.tar.gz -C /tmp ${LOG_FILENAME}.bin

# Check simulation results
set -e
set -x
# Workaround to achieve allow-failure
if [ "${MOTION_TYPE}" == "WalkingOnStairs" ] && [ "${MPC_FRAMEWORK}" == "OnlineMpc" ]; then
  ALLOW_FAILURE=true
else
  ALLOW_FAILURE=false
fi
echo "- ALLOW_FAILURE: ${ALLOW_FAILURE}"
if [ "${MOTION_TYPE}" == "WalkingOnPlane" ]; then
  EXPECTED_BASE_POS="1.34 -0.41 0.8"
elif [ "${MOTION_TYPE}" == "WalkingOnStairs" ]; then
  EXPECTED_BASE_POS="1.75 0.0 1.68"
else
  EXPECTED_BASE_POS="2.5 -0.25 0.8"
fi
python3 ${HOME}/catkin_ws/src/BaselineWalkingController/.github/workflows/scripts/checkSimulationResults.py /tmp/BWC-log-${RESULTS_POSTFIX}.bin --expected-base-pos ${EXPECTED_BASE_POS} || ${ALLOW_FAILURE}
