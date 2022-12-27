#! /usr/bin/env python

import sys
import argparse
import numpy as np
import eigen as e
import mc_log_ui

exit_status = 0

# Parse arguments
parser = argparse.ArgumentParser(description="Check simulation results.")
parser.add_argument("log-filename", type=str, help="log filename")
parser.add_argument("--tilting-angle-thre", type=float, default=30.0,
                    help="tilting angle threshold [deg]")
parser.add_argument("--expected-base-pos", nargs=3, type=float, default=None,
                    help="expected base position [m]")
parser.add_argument("--base-pos-thre", nargs=3, type=float, default=[0.5, 0.5, 0.5],
                    help="base position threshold [m]")
args = parser.parse_args()

# Load log file
args.log_filename = sys.argv[1]
print("[checkSimulationResults.py] Load {}".format(args.log_filename))
log = mc_log_ui.read_log(args.log_filename)

# Check tilting angle
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

if max_tilting_angle <= args.tilting_angle_thre:
    print(u"[success][checkSimulationResults.py] max_tilting_angle is below the threshold: {:.1f} <= \u00b1 {:.1f} [deg]".format(
        max_tilting_angle, args.tilting_angle_thre))
else:
    print(u"[error][checkSimulationResults.py] max_tilting_angle exceeds the threshold: {:.1f} > \u00b1 {:.1f} [deg]".format(
        max_tilting_angle, args.tilting_angle_thre))
    exit_status = 1

# Check last position
last_base_pos = np.array([log["FloatingBase_position_x"][-1],
                          log["FloatingBase_position_y"][-1],
                          log["FloatingBase_position_z"][-1]])

if args.expected_base_pos is not None:
    if (np.abs(last_base_pos - np.array(args.expected_base_pos)) < np.array(args.base_pos_thre)).all():
        with np.printoptions(precision=2):
            print(u"[success][checkSimulationResults.py] last_base_pos is within the expected range: {} <= {} \u00b1 {} [m]".format(
                last_base_pos, np.array(args.expected_base_pos), np.array(args.base_pos_thre)))
    else:
        with np.printoptions(precision=2):
            print(u"[error][checkSimulationResults.py] last_base_pos is outside the expected range: {} > {} \u00b1 {} [m]".format(
                last_base_pos, np.array(args.expected_base_pos), np.array(args.base_pos_thre)))
        exit_status = 1

sys.exit(exit_status)
