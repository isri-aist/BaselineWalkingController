#! /usr/bin/env python

import time
import mc_rtc
import mc_control

time.sleep(15)

cli = mc_control.ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc")

time.sleep(3)

elem = mc_control.ElementId(["BWC", "RosWalk"], "SetGoal")
cli.send_request(elem)

time.sleep(3)

elem = mc_control.ElementId(["BWC", "RosWalk"], "WalkToGoal")
cli.send_request(elem)
