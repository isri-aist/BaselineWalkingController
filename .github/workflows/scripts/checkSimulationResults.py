#! /usr/bin/env python

import sys
import numpy as np
import eigen as e
import mc_log_ui

log_filename = sys.argv[1]
print("[checkSimulationResults.py] Load {}".format(log_filename))
log = mc_log_ui.read_log(log_filename)

quat_data_list = np.array([log["FloatingBase_orientation_w"],
                           log["FloatingBase_orientation_x"],
                           log["FloatingBase_orientation_y"],
                           log["FloatingBase_orientation_z"]]).T

tilting_angle_list = []
for quat_data in quat_data_list:
    # Inverse is required because the left-hand system is used in mc_rtc
    quat = e.Quaterniond(*quat_data).inverse()
    rot = quat.toRotationMatrix()
    z_axis = e.Vector3d(rot.block(0, 2, 3, 1))
    tilting_angle_list.append(np.rad2deg(np.arccos(z_axis.dot(e.Vector3d.UnitZ())))) # [deg]
tilting_angle_list = np.array(tilting_angle_list)

max_tilting_angle = np.max(np.abs(tilting_angle_list))

tilting_angle_thre = 20.0 # [deg]
if max_tilting_angle <= tilting_angle_thre:
    print("[checkSimulationResults.py] max_tilting_angle is below the threshold: {:.1f} <= {:.1f} [deg]".format(
        max_tilting_angle, tilting_angle_thre))
    sys.exit(0)
else:
    print("[checkSimulationResults.py] max_tilting_angle exceeds the threshold: {:.1f} > {:.1f} [deg]".format(
        max_tilting_angle, tilting_angle_thre))
    sys.exit(1)
