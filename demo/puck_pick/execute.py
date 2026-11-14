#!/usr/bin/env python3
"""Throwaway demo: pick the planned puck on the REAL robot and load it into the clock, via control_master.
--unload (sis2): nominal -> via -> over the stored puck -> down -> close -> up -> via -> place on the box -> nominal,
one timed move each (taught poses, clock not turned).
--dice [N]: dice face N (1-6, default 6) from the stored pucks, one per clock slot (DICE_*), back to the nominal pose.
--goto nominal|via: one move to that pose (elbow first, shoulder last), nothing else.
--lying picks the lying puck of plan_lying.yaml (plan.py --lying) on its long axis and loads it the same way.

Sends craft_interfaces/TaskCommand on /planning/task_command one step at a time and waits for
/control/task_result. Keeps the /planning/hold heartbeat alive (control rejects tasks without it).
Press Enter before every step; Ctrl+C sends STOP and hold.
Usage:  source ~/craft/install/setup.bash && python3 ~/craft/demo/puck_pick/execute.py [--unload | --dice | --lying | --goto nominal|via] [--dry-run]
Delete with: rm -rf ~/craft/demo
"""
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from craft_interfaces.msg import ControlStatus, TaskCommand, TaskResult
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Bool

import plan  # URDF joint limits
import teach  # raw motor degrees -> MANIPULATOR target

HERE = Path(__file__).resolve().parent
ARM = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']
STATUS = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED', 3: 'STOPPING', 4: 'STOPPED', 5: 'REJECTED'}
# Loading pose set by hand (2026-11-03 14:32, arm relaxed and held there 35 s, ~/bags/final_1103_1432 mean),
# raw motor degrees through that day's offsets, kept as URDF joint degrees shoulder, elbow, wrist_pitch, tool_roll
# so they survive re-zeroing:
#   LM -151.7, shoulder 87.3 (+-0.9 by hand), elbow 122.7, wrist_pitch 229.2, tool_roll 88.6 (not used, below)
# Load 1 folds wrist_pitch there, load 2 the elbow and LM (shoulder stays where the ascend left it), load 3 the shoulder.
# The roll keeps its grasp angle until the drop, then turns 90 deg (back 1).
# (Before: 14:29 bag 92.8 / 108.9 / 219.0; a 13:55 reading elbow 317.9 / shoulder 54.1, wrong; measured load 1
# [48.85, 15.91, -115.5, -1.1] then load 2 [14.55, 18.71, -114.7, -2.2] + wrist 3 deg.)
# Clock slot 12 sits at the loading guide.
LOAD_URDF_DEG = [15.68, 4.85, -107.42, 0.0]  # tool_roll 0 = raw 90.9, as at the grasp
# LM at the clock: raw 1031.0 deg lines the gripper up with slot 12 (2026-11-05, guide moved, measured by hand;
# was -151.8 on the old guide with the hand-set loading pose, before that -99.9, 0.00296 = raw -28.6).
# The commanded value is what it reaches here.
LOAD_SLIDE = 0.0052  # [m] before LM_BACKLASH; commanded slide 0.0072 m = raw 548.3 deg (clock 12, 2026-11-07; was -0.01019,
                       # LM and clock re-aligned; was 829.7, before the clock reassembly 1031.0)
# In the clock the jaws open only to this gripper motor angle (measured; full open 94 deg would hit the slot).
RELEASE_GRIPPER_DEG = 147.9
# After the drop: tool_roll to raw 0.0 deg (URDF, offset 90.9 deg); then elbow and wrist half way to the start pose
# with the LM, shoulder and roll held, then everything to the start (2026-11-03: going straight to the start, the LM
# ran while the elbow was still low and the arm jammed on the clock. Was wrist alone to raw 220.8, ~8 deg; then
# elbow, wrist, roll all the way before the LM and shoulder).
BACK_ROLL_URDF_DEG = -90.9
LOAD_SLOT = 12
# bigbro --slot N: the clock slot this run loads into (1-12; the clock turns it to the loading guide, the arm poses
# stay the slot-12 ones). bigbro2 loads two pucks into 12, then 11.
# --slot 12,11 with --plan a.yaml,b.yaml (bigbro2): several pucks in one run, see bigbro_steps.
BIGBRO_SLOTS = [int(v) for v in sys.argv[sys.argv.index('--slot') + 1].split(',')] if '--slot' in sys.argv else [LOAD_SLOT]
BIGBRO_SLOT = BIGBRO_SLOTS[0]
# Nominal pose (fixed 2026-11-05): carriage lined up with clock 12, raw motor degrees LM, shoulder, elbow,
# wrist_pitch, tool_roll, gripper. Every run starts here and ends here. Reassembled 2026-11-06 (new elbow
# bracket); LM 858.3 = clock 12 after LM and clock were re-aligned (2026-11-06, was 829.7, 941.1); was
# [1030.9, 69.3, 181.0, 195.8, 90.3, 146.7].
# 2026-11-07: timing belt re-fitted and the LM scale re-measured (control.yaml): clock 12 = LM 548.3 (measured); the
# box places are their slide positions [m] in raw degrees again (box centre = LM mid travel 594.1).
# 2026-11-13: re-set by hand, this is now the nominal pose: DYNAMIXEL Wizard positions, LM the guide-side reading (checkarm 561.4,
# Wizard 201.7 + 1 turn); tool_roll Wizard 360 = 0.0 (limit -175..175). Was [548.3, 84.9, 100.1, 16.8, 90.0, 94.0].
NOMINAL_RAW_DEG = [561.4, 53.9, 178.0, 58.4, 0.0, 73.1]
# Via pose between the nominal pose and any work, both ways. Shoulder, elbow, wrist_pitch given by hand
# (2026-11-06); LM the old via - nominal difference on the new nominal; tool_roll and gripper as the nominal pose
# (they must not move); LM = nominal (858.3 since 2026-11-06, was 829.7, 940.8). Was [1030.6, 83.4, 101.2, 114.2, 91.8, 149.6].
# 2026-11-13: re-set with teacharm jog (teach.yaml p12, LM held at the new nominal 561.4) next to the new nominal pose.
# Was [548.3, 92.5, 182.9, 104.6, 90.0, 141.9].
VIA_RAW_DEG = [561.4, 93.3, 177.6, 61.3, -0.4, 73.1]
# Carrying a puck out of the clock (sis2 8, dice): the 2026-11-06 via (wrist folded, tool_roll 90), LM as the via.
# The new via above stands the tool up: on the way from over the slot the puck ends rose to 0.535 m and hit the
# structure (2026-11-13, user); through this pose they stay under 0.402 m (FK, 60 mm puck) and tool_roll does not turn.
CARRY_VIA_RAW_DEG = [VIA_RAW_DEG[0], 92.5, 182.9, 104.6, 90.0, 141.9]
# On the way there the shoulder first goes this much further out (away from the folded nominal side, + URDF), then back
# to the carry via (2026-11-13, user: the puck end grazed the structure the gripper rests on at the nominal pose,
# its underside 65 mm over the battery).
CARRY_SHOULDER_OUT_DEG = 5.0
# sis2 (taught 2026-11-06, clock reassembled, slot 12 at the guide; LM taught at 829.7, now the clock-12 LM 858.3): raw motor degrees LM, shoulder, elbow, wrist_pitch,
# tool_roll, gripper. Pick via: over the clock, tool_roll 0, way in and out. Pre-grasp: jaws part open around the
# stored puck, 28 mm down from its top. Lift: gripped and lifted clear of the slot. The gripper values are what the
# readings showed; the moves leave the gripper alone (GRIPPER steps open / close it). tool_roll set to exactly 0 / 90
# (read 1.0 / 91.1).
PICK_VIA_RAW_DEG = [548.2, 121.8, 97.0, 17.1, 0.0, 134.3]  # jog-taught grasp_via 15:51; LM as the approach (was 872.0)
# grasp wrist +0.5 deg on the robot (2026-11-07; was 132.46)
PRE_GRASP_RAW_DEG = [548.2, 92.06, 118.9, 132.96, 90.0, 90.0]  # re-taught 2026-11-06 evening (92.7, 116.0, 132.2, roll
                                                              # read 1.1), wrist -2 deg toward the carousel centre, then
                                                              # 3 mm straight up, tool angle kept (URDF model)
PICK_LIFT_RAW_DEG = [548.2, 104.2, 137.8, 146.3, 0.0, 168.4]  # jog-taught grasp_ascend 16:00; LM as the approach (was 871.9)
# Travel time per sis2 step [s] (time-based profile, firmware 2026-11-06: the LM and arm joints of a step start and
# arrive together). Not listed or 0: as fast as the speed caps allow (69 deg/s arm, 41 gripper, 28 mm/s LM; 2026-11-07), the farthest
# joint decides. A time shorter than the caps allow is stretched to them.
SLOW_SECONDS = {}  # step label -> time [s], filled from the real start pose in main() for the nominal <-> via moves
GOTO_DEG_S = 10.0  # (user 2026-11-14 "speed up": was 5.2) into the nominal pose: joint speed [deg/s] (the speed cap is 69), the farthest joint decides
VIA_DEG_S = 20.0  # (user 2026-11-14 "speed up": was 12) moves to a via pose: faster (wrist swings of 100+ deg took 20 s at 5.2), still time-based
APPROACH_DEG = 4.0  # nominal <-> via: the last this many degrees of the farthest joint are a separate slow move
APPROACH_DEG_S = 1.5  # speed of that final approach [deg/s]
APPROACH_MIN_S = 2.0
GOTO_MIN_S = 3.0  # nominal <-> via: no move shorter than this [s]
STEP_SECONDS = {
    'sis2 1:': 0.0,  # nominal -> nominal via
    'sis2 2:': 0.0,  # -> over the stored puck (tool_roll, gripper)
    'sis2 4:': 0.0,  # down to the grasp
    'sis2 6:': 0.0,  # up 20 mm
    'sis2 7:': 0.0,  # up to 40 mm
    'sis2 8:': 0.0,  # -> nominal via
    'sis2 9:': 0.0,  # -> over the place pose
    'sis2 10:': 0.0,  # descend onto the box
    'sis2 12:': 0.0,  # tool_roll 90 deg after the release
    'sis2 13:': 0.0,  # ascend
    'sis2 14:': 0.0,  # -> nominal
    # Dice: the put-down slow (2026-11-07, user): at the caps it takes 0.3-0.8 s for the 75 mm and the joints go
    # past the touch and press the puck into the box; 2 s = ~37 mm/s down, the time-based ramp easing into the touch.
    'put down on the': 2.0,
    # bigbro: down onto the puck in front, 40 mm in 2 s (2026-11-14, user: the jaws went too deep and hit the box).
    # At the caps (0 here) the joints run past the goal before they settle, as the dice put-down did.
    'bigbro pick: descend': 2.0,  # per pose of descent_path when it has several: see step_seconds
}
# Place on the box: the taught touching pose (857.6, 94.0, 283.1, -59.7, 90; it pressed a little) moved in the URDF
# model to the box centre (x -12.5 mm, 112.5 mm ahead of the LiDAR face: +19.3 mm ahead, -20.4 mm along the LM) and
# 2 mm up, tool angle kept, then 10 mm more (it still pressed into the box). Offsets from the straight-up pose
# (2026-11-06 evening; the earlier 1044.1, 110.3, 261.7, -64.8 used the old elbow/shoulder offsets and went ~37 mm too
# far; then 99.8, 275.0, -62.1 pressed).
PLACE_RAW_DEG = [593.2, 99.5, 269.3, -68.1, 90.0]
# 20 mm straight over it (30 over the earlier place; model), tool angle kept: come down from here, open, go back up.
PLACE_ABOVE_RAW_DEG = [593.2, 99.9, 256.6, -80.3, 90.0]
# After the release the jaws turn 90 deg off the puck (as the clock load does, sis1 back 1) before going up.
PLACE_RELEASE_ROLL_RAW_DEG = 0.0
# Dice (--dice [N], 2026-11-07): the stored pucks of the clock one after the other onto the box as dice face N (1-6,
# default 6). 3x3 grid on the box centre, 60 mm apart. Box centre on the LM middle (2026-11-07, user: fits the
# rules best; LM 594.1, gripper at LiDAR centre -12.65 mm; was -7.5 on the clock-12 line, LM 548.3, before that
# -12.5), 112.5 mm ahead of the LiDAR face: rows 172.5 / 112.5 / 52.5 mm ahead. Columns +-60 mm are past the LM ends
# (puck centre reach -63.0 .. +37.7 mm), so the side columns sit at the LM ends instead, both 9.65 mm off the 60 mm
# grid (the rules allow 10; with the box on the clock-12 line left was 14.8, right 4.5). Computed in the URDF model from the centre place pose (box top + 12 mm), the held puck's bottom kept on the
# vertical line over its pip: place, 40 mm and 75 mm over it (75: the puck bottom 15 mm over the placed pucks' tops;
# the tool leans up to 17 deg there, the wrist limits). Each row aims 26 mm farther out than its target (puck 1 landed
# 26 mm short, 2026-11-07). Jaws close left-right (tool_roll raw 0); no roll turn after the release (the open jaws
# would sweep 64 mm into the neighbours). Raw shoulder, elbow, wrist_pitch per row and height; raw LM per column.
# Row heights (2026-11-07 run sis_1107_0405: the middle row sets the puck down just right, the near row lets it fall
# from 2 mm, the far row presses it 3 mm into the box; the shoulder springs back +0.7-0.9 deg at the far release):
# far 0 / 40 moved 3 mm up, near 0 / 40 / 75 2 mm down, tool angle and the vertical line kept (URDF model); far 0
# 2 mm more (5 mm in all; 2026-11-07, user: it need not touch, a puck let go that low stands; was [131.28, 209.48,
# -96.16]; far 40 stays +3, unused since the 40 mm stop went and +5 is past the wrist_pitch limit). Far 75
# stays (3 mm up is past the wrist_pitch limit; it is only the clearance height). Was far 0 [130.43, 213.48, -93.01],
# 40 [122.4, 206.69, -99.83]; near 0 [88.49, 281.42, -67.01], 40 [90.36, 255.07, -91.49], 75 [84.93, 237.75, -105.24].
# Front-back error fed forward (2026-11-07 run sis3 6, six pucks; DICE_EY_MM): every pose of a row moved toward the
# robot by its ey (URDF model, height and tool angle kept, so the vertical line too). Was far 0 [132.0, 206.38,
# -98.54], 40 [124.31, 199.82, -104.79], 75 [113.7, 205.16, -101.06]; middle 0 [107.02, 258.68, -71.22], 40 [110.62,
# 227.68, -98.61], 75 [100.79, 222.06, -103.07]; near 0 [88.53, 282.59, -65.8], 40 [90.13, 256.58, -90.21], 75 [84.41,
# 239.76, -103.75]. Far 0 then 2 mm up (2026-11-07 run sis_1107_1343: the far put-down was held 2.0 mm over its
# target before the release (it touched ~1.5 mm earlier than in sis_1107_1237/1246, middle and near as before) and
# dropped 9.6 mm under it after; was [127.88, 216.12, -92.92]), then 1 mm more (user; was [128.37, 213.68, -94.87]).
# 2026-11-14 21:4x (user, teacharm jog with a puck in the jaws): the 3x3 place poses on the robot. Taught p13 (1, far
# left), p14 (4, middle left), p15 (7, near left), p16 (9, near right); the rest filled in (teach.yaml p17 = 6, p18 = 3):
# a row keeps its taught arm pose, a column its LM; the centre column LM is the mean of the left and right ones.
# 40 / 75 mm: computed in the URDF model straight over each place pose, the tool at the place angle where the joints
# allow (far 40: +3.5 deg, far 75: +15, middle 75: +7.5, near 75: +4 deg lean; 40 mm middle / near square).
# Place (0) raised 15 mm over the taught poses (user 2026-11-14; tool angle kept, model): taught far 112.8, 238.9,
# -73.5 / middle 96.9, 267.4, -61.5 / near 84.4, 283.8, -59.0.
DICE_ROWS = {
    'far': {0: [115.0, 225.23, -84.97], 40: [117.58, 203.59, -100.54], 75: [111.8, 196.69, -101.72]},
    'middle': {0: [97.45, 257.61, -70.73], 40: [100.38, 237.77, -87.65], 75: [100.36, 215.89, -102.05]},
    'near': {0: [84.62, 274.57, -68.02], 40: [86.61, 257.32, -83.27], 75: [89.31, 231.59, -102.3]},
}
DICE_COLS_LM_RAW_DEG_TAUGHT = {'left': 141.3, 'centre': 593.35, 'right': 1045.4}
DICE_ROWS_1107_1417 = {
    'far': {0: [128.65, 212.37, -95.9], 40: [119.75, 210.33, -98.83], 75: [110.2, 212.18, -97.54]},
    'middle': {0: [103.7, 263.61, -69.61], 40: [106.61, 234.35, -95.95], 75: [97.07, 227.14, -101.71]},
    'near': {0: [88.22, 282.87, -65.83], 40: [89.83, 256.87, -90.22], 75: [84.13, 239.95, -103.84]},
}
DICE_ROWS_0329 = {
    'far': {0: [130.43, 213.48, -93.01], 40: [122.4, 206.69, -99.83], 75: [113.7, 205.16, -101.06]},
    'middle': {0: [107.02, 258.68, -71.22], 40: [110.62, 227.68, -98.61], 75: [100.79, 222.06, -103.07]},
    'near': {0: [88.49, 281.42, -67.01], 40: [90.36, 255.07, -91.49], 75: [84.93, 237.75, -105.24]},
}
DICE_COLS_LM_RAW_DEG_0329 = {'left': 142.0, 'centre': 548.3, 'right': 1046.2}
# Placement error as formulas (2026-11-07 run sis3 6 choi, six pucks, measured on the box; the box is not quite flat
# and a released puck can shift, ~1 mm). x = toward the LM motor, y = away from the LM, both from where the puck was
# aimed [mm]; xc = aimed puck centre toward the LM motor from the LiDAR centre [mm]; t = (row ahead - 112.5) / 60
# (near -1, middle 0, far +1).
#   ex = -9.83 - 0.0298 (xc - 12.65)    away end -8.3, motor end -11.3; no row trend (+0.5 per row), rms 0.9
#   ey = 10.5 + 2.75 t - 6.75 t^2       near +1.0, middle +10.5, far +6.5; no column trend (+0.3), rms 0.7
# ex is fed forward on the LM (dice_lm), ey on the row poses (DICE_ROWS, moved once in the model). Measured pairs
# (ex, ey) far/middle/near: away end -8,7 / -7,10 / -10,0; motor end -11,6 / -12,11 / -11,2.
DICE_EX_MM = (-9.83, -0.0298, 12.65)  # a, b, xc0: ex = a + b (xc - xc0)
DICE_EY_MM = {'far': 6.5, 'middle': 10.5, 'near': 1.0}  # applied to DICE_ROWS above
# Box and grid (2026-11-07): the side columns cannot reach the 60 mm grid (the LM ends), so the box centre sits in
# the middle of where the pucks really land at the LM ends (with ex: -46.0 .. +51.7 mm, middle +2.8): both side
# columns land ~11.2 mm inside their grid points, the centre column on it. Was the LM middle (+12.65, LM 594.1, before
# ex was known), the clock-12 line (+7.5, LM 548.3), +12.5.
DICE_BOX_X_MM = 2.8   # box centre toward the LM motor from the LiDAR centre
DICE_GRID_MM = 60.0
DICE_COL_STEPS = {'left': -1, 'centre': 0, 'right': 1}  # robot left = away from the LM motor
# sis3 cal away|mid|motor: three pucks (far, middle, near) at an LM end / the LM middle with no ex correction, to
# re-measure ex (ey is in the rows, so y shows what is left). The columns are 50 mm apart: one per run.
DICE_CAL_LM_RAW_DEG = {'away': 142.0, 'mid': 594.1, 'motor': 1046.2}
DICE_PLACE_ROLL_RAW_DEG = 0.0  # jaws close left-right over the box
# Pips as (row, column), the picture seen from behind the robot: top row = far. 2 and 3 on the far-left / near-right
# diagonal. Put down column by column from the robot's left, far to near in each, so the held puck never passes over a
# placed one.
DICE_FACES = {
    1: [('middle', 'centre')],
    2: [('far', 'left'), ('near', 'right')],
    3: [('far', 'left'), ('middle', 'centre'), ('near', 'right')],
    4: [('far', 'left'), ('near', 'left'), ('far', 'right'), ('near', 'right')],
    5: [('far', 'left'), ('near', 'left'), ('middle', 'centre'), ('far', 'right'), ('near', 'right')],
    6: [('far', 'left'), ('middle', 'left'), ('near', 'left'), ('far', 'right'), ('middle', 'right'), ('near', 'right')],
}
# The face: a bare 1-6 or cal on the command line (sis3 4, sis3 4 choi, sis3 cal motor); default 6.
DICE_FACE = next((a if a == 'cal' else int(a) for a in sys.argv[1:] if a in ('1', '2', '3', '4', '5', '6', 'cal')), 6)
# sis3 cal away|mid|motor: one column of the bend map, far to near (default mid).
DICE_CAL_COL = next((a for a in sys.argv[1:] if a in DICE_CAL_LM_RAW_DEG), 'mid')
# Put down from the robot's side out (user 2026-11-14): near row first, then middle, then far; left before right.
# (Was column by column, far to near, so the held puck never passed over a placed one; at 75 mm the puck bottom
# clears the placed tops, model.)
_ROW_ORDER = {'near': 0, 'middle': 1, 'far': 2}
_COL_ORDER = {'left': 0, 'centre': 1, 'right': 2}
DICE_FACES = {k: sorted(v, key=lambda p: (_ROW_ORDER[p[0]], _COL_ORDER.get(p[1], 1))) for k, v in DICE_FACES.items()}
DICE_FACES['cal'] = [('far', DICE_CAL_COL), ('middle', DICE_CAL_COL), ('near', DICE_CAL_COL)]
# The clock brings the next stored puck to the guide one slot at a time (after each place, the arm over the box).
# ASSUMED order: slot 12 first, then 1, 2, ... (one slot each, +30 deg); change if the pucks sit the other way round.
DICE_SLOTS = [12, 1, 2, 3, 4, 5]
# Over the stored puck (2026-11-06 night): going straight from the via pose to the grasp swept the open jaws into
# the neighbouring pucks. Now down onto it and back up on the line over it (URDF model, offsets from the straight-up
# pose; fingertip within 0.6 mm of that line): 20 mm up with the tool vertical (the wrist reaches 146.8 of its 147.1
# limit there), 40 mm up leaning 14 deg (fingertip 12 mm over the puck tops; the jaws cover the top 28 mm).
# tool_roll stays at the via pose's 90 the whole pick (2026-11-06 night: coming straight down, the jaws need no turn;
# was 0). The roll turns about the tool axis, so the waypoints' positions do not change with it.
PICK_MID_RAW_DEG = [548.2, 88.23, 137.1, 146.83, 90.0]
PICK_ABOVE_RAW_DEG = [548.2, 98.87, 140.13, 146.5, 90.0]
# The pose the descent starts from: wrist_pitch 1.5 deg more than computed (2026-11-06 night, on the robot).
PICK_DESCEND_FROM_RAW_DEG = [548.2, 98.87, 140.13, 148.0, 90.0]
PICK_OPEN_GRIPPER_DEG = 94.0  # full open going in to the stored puck (2026-11-07; was 145.5, 134.3, 90, 152.9). The arm
                              # board's gripper takes 70 % of that move's time (firmware kTimeScale): open before the arm arrives
# Back toward the nominal pose the wrist_pitch arrives first, then the elbow (2026-11-05: the elbow must not finish
# before the wrist_pitch); wrist_first() inserts that wrist-only move. Out to the via pose it is the reverse:
# shoulder, then elbow (with LM, tool_roll, gripper), then wrist_pitch (out_to_via).
# Nominal <-> via in one unstaged move each way (2026-11-05: an elbow 20 deg lift first, then dropped; an overload
# fault came during it).
# The LM stops ~2 mm short of the puck line (backlash): aim this much further toward the LM motor
# (+slide; the motor end is about +0.063). Applies to every plan pose and to LOAD_SLIDE.
LM_BACKLASH = 0.002  # [m]
# Unload: the puck sits this far below where the loading pose lets it go.
UNLOAD_DEPTH = 0.03  # [m] straight down from the loading pose
# The plain swing to the loading pose ends by sliding in from behind ~2 cm over the loading height and the
# open jaws catch the stored puck. So: swing to UNLOAD_ABOVE over the grasp with the wrist already
# folded to its grasp angle (shoulder/elbow high), then straight down. Raising it leans the tool more
# (4 cm: 24 deg, 6 cm: 34 deg) and bows the descent more (2.4 / 5.0 mm).
UNLOAD_ABOVE = 0.06  # [m] above the unload grasp, i.e. 3 cm over the loading pose
# Put-down: open this far above the front grasp height. A puck taken from the clock sits deeper in the
# jaws than a front-grasped one, so lowering to the grasp height pressed it into the shelf (the arm's
# I gain kept pushing) and the arm dropped when the jaws let go. The puck now falls <= this far.
PUT_DOWN_ABOVE = 0.01  # [m]
# Elbow first, shoulder last (2026-10-27): every arm move runs elbow, wrist, roll and LM first with the shoulder
# held, then the shoulder alone, so the shoulder never moves with the rest of the arm still swinging (it carries
# the whole arm and ran out of torque). Not for taking the puck out of the clock (NO_STAGING).
# Vertical moves go in STAGE_SEG pieces so the staged path stays near the line.
# Vertical moves (descend / lift / put down / rise) go in one unstaged move (2026-11-02, was 4 mm pieces staged
# elbow-first: 20 moves for 4 cm). Joints arrive one by one, so the TCP bows ~10 mm front-back mid-move.
NO_STAGING = ('arm over the slot', 'straight down to the grasp', 'straight up ', 'descend', 'lift the puck',
              'put down', 'rise above the puck', 'sis2 ', 'dice ', 'bigbro ',
              'via pose:', 'via pose (off', 'to the nominal pose')  # out_to_via; the way back to the via pose stays staged
STAGE_SEG = 1000  # plan frames per vertical piece: one move
# --lying: the arm reaches the plan's tool angle through a pose LYING_HIGH over the grasp, then comes down the
# vertical line in 1 cm pieces. Going straight from the standing lift pose to the approach dipped the fingertips
# into the floor in simulation (2026-10-27, nearly happened on the robot).
LYING_HIGH = 0.10  # [m]
# The swings between the standing lift pose and LYING_HIGH move the TCP on a straight line in LYING_SWING staged
# pieces. Staged as one joint move, going out swept the fingertips to the floor (-1 mm) and coming back pressed
# the held puck 42 mm into the shelf; 20 pieces clear the shelf by >= 49 mm (simulated, 2026-10-27).
LYING_SWING = 20


def nominal_pose():
    """NOMINAL_RAW_DEG as a MANIPULATOR target (teach.raw_to_target)."""
    return teach.raw_to_target(NOMINAL_RAW_DEG)


def via_pose():
    """VIA_RAW_DEG as a MANIPULATOR target."""
    return teach.raw_to_target(VIA_RAW_DEG)


def carry_via_pose():
    """CARRY_VIA_RAW_DEG as a MANIPULATOR target (the way out of the clock with a puck)."""
    return teach.raw_to_target(CARRY_VIA_RAW_DEG)


def carry_via_path():
    """The carry via with the shoulder CARRY_SHOULDER_OUT_DEG further out, then the carry via (arm joints only)."""
    via = carry_via_pose()[:5]
    out = list(via); out[1] += math.radians(CARRY_SHOULDER_OUT_DEG)
    return [out, via]


def wrist_first(prev, target, what):
    """Step: prev with only the wrist_pitch moved to target's (the elbow follows in the next step)."""
    q = list(prev)
    q[3] = target[3]
    return (f'wrist_pitch to {what} first, the rest stays', TaskCommand.MANIPULATOR, q)


def close_but_shoulder(a, b, joint_deg=15.0, slide_m=0.005):
    """True when LM (slide) and the elbow, wrist_pitch and tool_roll of the poses a and b are within joint_deg / slide_m:
    only the shoulder differs, so the arm can go in one move (2026-11-13, user: the stop-and-go steps looked jerky)."""
    return abs(a[0] - b[0]) < slide_m and all(abs(a[i] - b[i]) < math.radians(joint_deg) for i in (2, 3, 4))


def out_to_via(prev):
    """Steps prev -> via pose, the reverse of the way back: the shoulder alone, then the elbow with LM, tool_roll and
    gripper, then the wrist_pitch alone."""
    via = via_pose()
    m = TaskCommand.MANIPULATOR
    if close_but_shoulder(prev, via):  # one smooth move instead of three stop-and-go ones
        return [('via pose: one move (only the shoulder moves)', m, via)]
    shoulder = list(prev[:1]) + [via[1]] + list(prev[2:])
    elbow = via[:3] + [prev[3]] + via[4:]
    return [('via pose: shoulder first, the rest stays', m, shoulder),
            ('via pose: elbow, LM, tool_roll, gripper next (wrist_pitch stays)', m, elbow),
            ('via pose (off the nominal pose): wrist_pitch last', m, via)]


def with_via(steps, start, out=True):
    """steps with, before the last move to start: the wrist_pitch to the via pose, the via pose, the wrist_pitch to
    start; and (out) the via pose as the first move. A step's target is a pose or, in steps_from_plan, a list of
    poses."""
    via = via_pose()
    nested = lambda arg: isinstance(arg[0], list)
    target = lambda arg: arg[-1] if nested(arg) else arg
    last = max(i for i, (_, kind, arg) in enumerate(steps)
               if kind == TaskCommand.MANIPULATOR and target(arg) == start)
    wrap = (lambda q: [q]) if nested(steps[last][2]) else (lambda q: q)
    prev = next(target(arg) for _, kind, arg in reversed(steps[:last]) if kind == TaskCommand.MANIPULATOR)
    if all(abs(a - b) < 1e-6 for a, b in zip(prev[:5], via[:5])):  # already comes back through the via pose
        return [(n, k, wrap(q)) for n, k, q in out_to_via(start)] + steps if out else steps
    name, kind, wrist = wrist_first(prev, via, 'the via pose')
    back = ('via pose before the nominal pose', TaskCommand.MANIPULATOR, wrap(via))
    name2, kind2, wrist2 = wrist_first(via, start, 'the nominal pose')
    steps = steps[:last] + [(name, kind, wrap(wrist)), back, (name2, kind2, wrap(wrist2))] + steps[last:]
    if out:
        steps = [(n, k, wrap(q)) for n, k, q in out_to_via(start)] + steps
    return steps


def load_pose():
    """LOAD_URDF_DEG in radians."""
    return [math.radians(v) for v in LOAD_URDF_DEG]


def below(arm, dz):
    """arm (shoulder, elbow, wrist_pitch, tool_roll) moved so the TCP goes dz straight down and the
    tool keeps its angle: Newton on the three parallel pitch joints, URDF FK."""
    ch = plan.Chain(plan.URDF)
    names = ARM[1:4]
    def pose(q):
        T = ch.fk(dict(zip(names, q), tool_roll_joint=arm[3]))
        return np.array([T[1, 3], T[2, 3], math.atan2(T[2, 2], T[1, 2])])
    q = np.array(arm[:3], dtype=float)
    goal = pose(q) - np.array([0.0, dz, 0.0])
    for _ in range(50):
        e = goal - pose(q)
        e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(pose(q + h) - pose(q)) / 1e-6 for h in np.eye(3) * 1e-6])
        q += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm below the loading pose')
    for n, v in zip(names, q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits '
                               f'{math.degrees(lo):.1f}..{math.degrees(hi):.1f} at {dz * 100:.1f} cm below the loading pose')
    return [float(v) for v in q] + [arm[3]]


def above(arm, dz):
    """arm with wrist_pitch and tool_roll kept, shoulder/elbow solved so the TCP sits dz straight above.
    The tool leans by however much shoulder+elbow change (the wrist limit forbids keeping it)."""
    ch = plan.Chain(plan.URDF)
    tcp = lambda q: ch.fk(dict(zip(ARM[1:], q)))[1:3, 3]
    q = np.array(arm, dtype=float)
    goal = tcp(q) + np.array([0.0, dz])
    for _ in range(50):
        e = goal - tcp(q)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(tcp(q + h) - tcp(q)) / 1e-6 for h in (np.r_[1e-6, 0, 0, 0], np.r_[0, 1e-6, 0, 0])])
        q[:2] += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm above the unload grasp')
    for n, v in zip(ARM[1:3], q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits at {dz * 100:.1f} cm above')
    return [float(v) for v in q]


PLAN_FILES = sys.argv[sys.argv.index('--plan') + 1].split(',') if '--plan' in sys.argv else ['plan.yaml']  # cam_pick.py: plan_cam.yaml
PLAN_FILE = PLAN_FILES[0]


def plan_pose(name=None):
    """pose(i): frame i of a plan.py output as [slide + LM_BACKLASH, shoulder, elbow, wrist_pitch, tool_roll]."""
    name = name or PLAN_FILE
    f = yaml.safe_load(open(HERE / name))['frames']
    if not f:  # plan.py writes no frames when a waypoint is outside the limits
        sys.exit(f'{name} has no frames: plan.py failed (a waypoint outside the joint/LM limits); nothing sent')
    return lambda i: [f[i]['joints'][ARM[0]] + LM_BACKLASH] + [f[i]['joints'][n] for n in ARM[1:]]


def frames_path(pose, i0, i1):
    """pose(i) for plan frames i0 -> i1 (either direction) every STAGE_SEG frames, both ends included."""
    step = STAGE_SEG if i1 > i0 else -STAGE_SEG
    return [pose(i) for i in list(range(i0, i1, step))[1:] + [i1]]


def descent_path(pose, name=None):
    """The descent of a plan (frames 20 -> 59, the TCP straight down) as a few poses instead of one joint move.
    2026-11-14 18:07 (bag squeeze5b): the approach pose leans the tool ~10 deg (the wrist limit binds up there) and one
    joint move spread that lean over the whole way down: the jaw on the robot side passed 12.8 mm from the puck wall
    still leaning 6 deg (model; straight: 19.2) and the puck was knocked over before the jaws closed. Poses: where the
    lean is gone (the wrist leaves its limit; the tips are still over the puck top), half-way from there, the grasp."""
    f = yaml.safe_load(open(HERE / (name or PLAN_FILE)))['frames']
    lean = [sum(f[i]['joints'][n] - f[59]['joints'][n] for n in ARM[1:4]) for i in range(60)]  # tool angle vs the grasp's
    knee = next((i for i in range(20, 59) if abs(lean[i]) < math.radians(0.5)), 59)
    if knee <= 21 or knee >= 57:  # no lean at the approach (or it lasts all the way down): one move, as before
        return [pose(59)]
    return [pose(knee), pose((knee + 59) // 2), pose(59)]


def lying_line(heights):
    """Arms (shoulder, elbow, wrist_pitch, tool_roll) with the plan_lying tool angle at each height over its grasp."""
    p = yaml.safe_load(open(HERE / 'plan_lying.yaml'))
    ch, roll = plan.Chain(plan.URDF), p['frames'][0]['joints']['tool_roll_joint']
    mg = math.radians(p['assume']['limit_margin_deg'])
    out = []
    for h in heights:
        for q in plan.ik_top_down(ch, np.array(p['puck_grasp_urdf']) + [0, 0, h], p['assume']['tool_tilt_deg']):
            if all(ch.limits(n)[0] + mg <= q[n] <= ch.limits(n)[1] - mg for n in ARM[1:4]):
                out.append([q[n] for n in ARM[1:4]] + [roll]); break
        else:
            raise RuntimeError(f'no pose {h * 100:.0f} cm over the lying grasp')
    return out


def tcp_path(arm0, arm1, n):
    """n arms (shoulder, elbow, wrist_pitch, tool_roll) moving the TCP on a straight line arm0 -> arm1 with the
    tool angle and roll turning evenly (IK per point, the branch nearest the previous point)."""
    ch = plan.Chain(plan.URDF)
    def pt(arm):
        T = ch.fk(dict(zip(ARM[1:], arm)))
        return T[:3, 3], (-90 - math.degrees(math.atan2(T[2, 1], T[1, 1]))) % 360
    (p0, t0), (p1, t1) = pt(arm0), pt(arm1)
    t1 = t0 + (t1 - t0 + 180) % 360 - 180
    out, prev = [], arm0
    for k in range(1, n + 1):
        f = k / n
        roll = arm0[3] + (arm1[3] - arm0[3]) * f
        sols = [[q[n_] for n_ in ARM[1:4]] + [roll] for q in plan.ik_top_down(ch, p0 + (p1 - p0) * f, t0 + (t1 - t0) * f)]
        sols = [q for q in sols if all(ch.limits(n_)[0] <= v <= ch.limits(n_)[1] for n_, v in zip(ARM[1:4], q))]
        if not sols:
            raise RuntimeError(f'no IK on the TCP line at {f:.0%}')
        prev = min(sols, key=lambda q: sum(abs(x - y) for x, y in zip(q, prev)))
        out.append(prev)
    out[-1] = list(arm1)
    return out


def stage(steps, prev):
    """Elbow first, shoulder last: each MANIPULATOR target (a pose or a path of poses) becomes the pose with the
    shoulder still at its previous angle, then the pose. prev: the arm pose before steps (None: first not staged)."""
    out = []
    for name, kind, arg in steps:
        if kind != TaskCommand.MANIPULATOR:
            out.append((name, kind, arg)); continue
        path = arg if isinstance(arg[0], list) else [arg]
        poses = []
        for q in path:
            mid = [q[0], prev[1]] + q[2:] if prev is not None and not name.startswith(NO_STAGING) else q
            if mid != q and mid != prev:
                poses.append(mid)
            poses.append(q); prev = q
        out.append((name, kind, poses))
    return out


def load_tail(lift, start):
    """Lift pose (puck held, LM on the puck line) -> clock slot LOAD_SLOT -> back to start."""
    load_slide = LOAD_SLIDE + LM_BACKLASH
    s = [('LM to the loading line, arm stays', TaskCommand.MANIPULATOR, [load_slide] + lift)]
    s.append((f'clock to slot {LOAD_SLOT} (loading guide)', TaskCommand.CLOCK, LOAD_SLOT))
    # Front to back in one joint move: the wrist swings ~250 degrees with the puck held.
    s.append(('arm to the loading pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + load_pose()))
    s.append(('open gripper (drop the puck into the clock)', TaskCommand.GRIPPER, False))
    s.append(('arm back to the lift pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + lift))
    if start is not None:  # reverse of the start: LM alone first, then the arm
        s.append(('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + lift))
        s.append(('arm back to the start pose', TaskCommand.MANIPULATOR, start))
    return s


def steps_from_plan(current_slide=None, start=None):
    """sis1, one move per step: the shoulder carries the whole arm, so going out the rest of the arm moves with the
    shoulder held, then the shoulder alone; loading and the way back follow the poses measured on 2026-11-03
    (LOAD/BACK_*). start None (dry run): the shoulder of the first move is the current one (nan)."""
    # plan.py layout at 20 Hz: 0-19 hold above, 20-59 descent, 60-79 close, rest hold (no lift).
    pose = plan_pose()
    approach, grasp = pose(0), pose(59)
    slide = LOAD_SLIDE + LM_BACKLASH
    load = [slide] + load_pose()
    fold = approach[:3] + [load[3]] + approach[4:]  # wrist_pitch first: elbow and wrist together scraped the shelf
    load1 = [slide, approach[1]] + load[2:]  # then the elbow and the LM; the shoulder stays at the ascend
    held = via_pose()[1] if start is not None else math.nan  # the run starts at the via pose
    # The gripper opens fully during move 1: a sixth joint 'gripper' in a MANIPULATOR task (control_master).
    full_open = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']['gripper_open_angle']
    s = [('move 1: LM to the puck line, arm but the shoulder above the puck, gripper full open',
          TaskCommand.MANIPULATOR, [[approach[0], held] + approach[2:] + [full_open]]),
         ('move 2: shoulder above the puck', TaskCommand.MANIPULATOR, [approach]),
         ('descend', TaskCommand.MANIPULATOR, descent_path(pose)),
         ('close gripper (grasp by current >= 0.1 A)', TaskCommand.GRIPPER, True),
         ('ascend', TaskCommand.MANIPULATOR, [approach]),
         ('load 1: wrist_pitch to the loading pose', TaskCommand.MANIPULATOR, [fold]),
         ('load 2: elbow and LM to the loading pose (shoulder stays)', TaskCommand.MANIPULATOR, [load1]),
         # control runs one task at a time, so the clock goes between the two: load 3 starts once it reports done
         (f'clock to slot {LOAD_SLOT} (loading guide)', TaskCommand.CLOCK, LOAD_SLOT),
         ('load 3: shoulder to the loading pose', TaskCommand.MANIPULATOR, [load]),
         (f'open gripper to {RELEASE_GRIPPER_DEG:g} deg (drop the puck into the clock)', TaskCommand.GRIPPER,
          math.radians(RELEASE_GRIPPER_DEG))]
    roll = load[:4] + [math.radians(BACK_ROLL_URDF_DEG)]
    s.append(('back 1: tool_roll to raw 0 deg', TaskCommand.MANIPULATOR, [roll]))
    if start is not None:
        # 2026-11-13 (user): from the clock straight to the via pose, then the nominal pose. The old way back (elbow
        # and wrist half way to the start) passed the gripper rest once the nominal pose changed and hit it.
        s.append(('back 2: to the nominal via pose, one move (LM moves 1.5 mm only)', TaskCommand.MANIPULATOR,
                  [via_pose()[:5]]))
        s.append(('back 3: LM, shoulder, elbow, wrist_pitch, tool_roll to the start pose', TaskCommand.MANIPULATOR,
                  [start]))
    return s


def lying_steps_from_plan(current_slide=None, start=None):
    """--lying: grip the lying puck's near end at the plan_lying tool angle, come down and lift it on the vertical
    line, swing it via the LYING_HIGH pose to the standing plan's lift pose (tool down, roll back to 0; the puck
    sits in the jaws as a standing-grasped one) and load it like steps_from_plan."""
    pose, lift = plan_pose('plan_lying.yaml'), plan_pose()(20)[1:]
    a = yaml.safe_load(open(HERE / 'plan_lying.yaml'))['assume']['approach_above']
    down = lying_line(np.arange(LYING_HIGH, a, -0.01))   # LYING_HIGH .. 1 cm above the approach
    high, slide = down[0], pose(0)[0]
    s = []
    if current_slide is not None:
        s.append(('arm to the standing lift pose, LM stays', TaskCommand.MANIPULATOR, [current_slide] + lift))
        s.append((f'tool out, {LYING_HIGH * 100:.0f} cm over the grasp height, LM stays', TaskCommand.MANIPULATOR,
                  [[current_slide] + q for q in tcp_path(lift, high, LYING_SWING)]))
    s.append(('open gripper', TaskCommand.GRIPPER, False))
    s.append(('LM to the puck line, arm stays', TaskCommand.MANIPULATOR, [slide] + high))
    s.append(('down to the approach (4 cm over the grasp)', TaskCommand.MANIPULATOR,
              [[slide] + q for q in down[1:]] + [pose(0)]))
    s.append(('descend', TaskCommand.MANIPULATOR, frames_path(pose, 20, 59)))
    s.append(('close gripper (grasp by current >= 0.1 A)', TaskCommand.GRIPPER, True))
    s.append(('lift the puck 4 cm', TaskCommand.MANIPULATOR, frames_path(pose, 59, 20)))
    s.append((f'up to {LYING_HIGH * 100:.0f} cm, LM stays', TaskCommand.MANIPULATOR, [[slide] + q for q in down[::-1]]))
    s.append(('arm to the standing lift pose (tool down, roll back), LM stays', TaskCommand.MANIPULATOR,
              [[slide] + q for q in tcp_path(high, lift, LYING_SWING)]))
    return s + load_tail(lift, start)


def pick_steps(n):
    """Take the stored puck at the clock guide out: over it, down, close, up (sis2 / dice). n: step label prefix."""
    approach = teach.raw_to_target(PRE_GRASP_RAW_DEG)[:5]
    over = teach.raw_to_target(PICK_ABOVE_RAW_DEG + [0.0])[:5]
    start_down = teach.raw_to_target(PICK_DESCEND_FROM_RAW_DEG + [0.0])[:5]
    m = TaskCommand.MANIPULATOR
    return [
        (f'{n} over the stored puck (40 mm, wrist +1.5), tool_roll stays {PICK_ABOVE_RAW_DEG[4]:g}, gripper open to '
         f'{PICK_OPEN_GRIPPER_DEG:g} deg', m, start_down + [math.radians(PICK_OPEN_GRIPPER_DEG)]),
        (f'{n} down to the grasp, one move (no 20 mm stop: fingertip within 2.4 mm of the line)', m, approach),
        (f'{n} close gripper (grasp the stored puck)', TaskCommand.GRIPPER, True),
        # One move (2026-11-07, the 20 mm stop dropped): TCP within 2.4 mm of the straight line, as the way down.
        (f'{n} up to 40 mm over the slot, one move', m, over),
    ]


def dice_lm(col):
    """URDF slide [m] for a column and a note for the step name. Grid columns: the grid point (DICE_BOX_X_MM +
    DICE_GRID_MM steps) with ex fed forward (DICE_EX_MM), kept inside the LM limits (an LM end cannot go further
    out: the note says how far it lands from its grid point). cal columns: their LM point as it is."""
    ch = plan.Chain(plan.URDF)
    # puck centre toward the LM motor from the LiDAR centre = slide - x0 (the arm moves in the plane of the shoulder)
    x0 = ch.origin('slide_joint')[0] + ch.origin('shoulder_joint')[0] - plan.ASSUME['lidar_center_x']
    if col in DICE_COLS_LM_RAW_DEG_TAUGHT and '--dice-ex' not in sys.argv:  # taught 2026-11-14 (no ex: taught on the robot)
        slide = teach.raw_to_target([DICE_COLS_LM_RAW_DEG_TAUGHT[col], 0, 0, 0, 0, 0])[0]
        return slide, f', LM raw {DICE_COLS_LM_RAW_DEG_TAUGHT[col]:g} (taught 11-14)'
    if col in DICE_COLS_LM_RAW_DEG_0329 and '--dice-0329' in sys.argv:  # 03:29 columns (user 2026-11-14)
        slide = teach.raw_to_target([DICE_COLS_LM_RAW_DEG_0329[col], 0, 0, 0, 0, 0])[0]
        return slide, f', LM raw {DICE_COLS_LM_RAW_DEG_0329[col]:g} (11-07 03:29)'
    if col in DICE_CAL_LM_RAW_DEG:
        slide = teach.raw_to_target([DICE_CAL_LM_RAW_DEG[col], 0, 0, 0, 0, 0])[0]
        return slide, f', x {(slide - x0) * 1000:+.1f} mm, no ex'
    a, b, xc0 = DICE_EX_MM
    goal = DICE_BOX_X_MM + DICE_GRID_MM * DICE_COL_STEPS[col]
    aim = (goal - a + b * xc0) / (1 + b)  # aim + ex(aim) = goal
    lo, hi = sorted(teach.raw_to_target([math.degrees(v), 0, 0, 0, 0, 0])[0] for v in teach.motor_limits()[0])
    slide = float(min(max(aim / 1000 + x0, lo), hi))
    x = (slide - x0) * 1000
    land = x + a + b * (x - xc0)
    note = f', x {goal:+.1f} mm (LM aim {x:+.1f})'
    return slide, note + (f', LM end: lands {land - goal:+.1f} mm off' if abs(land - goal) > 0.05 else '')


DICE_LINE_STEP = 0.015    # [m] vertical line over a pip in pieces this tall (5 mm: 13 stop-and-go moves, too slow, user 2026-11-14)
DICE_FINAL_DROP = 0.010   # [m] the last part down onto the box: its own timed step with contact detection
DICE_TILT_MAX_DEG = 30.0


def dice_line(lm, row):
    """2026-11-14 (user: the tool must stay square to the box while it puts a puck down). Poses from the 75 mm pose
    down the vertical line over the pip to the place pose: the TCP keeps the place pose's x / y and the tool keeps
    its angle (2.2 deg off vertical) wherever the joints allow (margin 3 deg); higher up it leans the least that fits
    and straightens on the way down. Model (URDF): near row square from 55 mm, middle from 45, far only the last 5 mm
    (arm near full reach). Returns (way down: line top .. DICE_FINAL_DROP, the final pose), each [slide, s, e, w, roll]."""
    ch = plan.Chain(plan.URDF)
    names = plan.Chain.NAMES
    q0 = [lm] + teach.raw_to_target([0] + DICE_ROWS[row][0] + [DICE_PLACE_ROLL_RAW_DEG, 0])[1:5]
    q75 = [lm] + teach.raw_to_target([0] + DICE_ROWS[row][75] + [DICE_PLACE_ROLL_RAW_DEG, 0])[1:5]
    fk = lambda q: ch.fk(dict(zip(names, [q[0] - LM_BACKLASH] + list(q[1:5]))))
    p0 = fk(q0)[:3, 3]
    top = fk(q75)[:3, 3][2] - p0[2]
    mg = math.radians(3.0)
    ok = lambda q: all(ch.limits(n)[0] + mg <= v <= ch.limits(n)[1] - mg for n, v in zip(names[1:4], q[1:4]))
    # the place pose's own tool angle in plan.ik_top_down terms
    tilt0 = min((t / 10 for t in range(-300, 301)), key=lambda t: min(
        (max(abs(sol[n] - q0[i + 1]) for i, n in enumerate(names[1:4])) for sol in plan.ik_top_down(ch, p0, t)),
        default=9.0))
    prev, out = q0, []
    for h in [DICE_FINAL_DROP + DICE_LINE_STEP * k for k in range(int(round((top - DICE_FINAL_DROP) / DICE_LINE_STEP)) + 1)]:
        best = None
        for d in [x / 2 for x in range(0, int(DICE_TILT_MAX_DEG * 2) + 1)]:
            for t in ((tilt0 + d, tilt0 - d) if d else (tilt0,)):
                for sol in plan.ik_top_down(ch, p0 + [0, 0, h], t):
                    q = [lm] + [sol[n] for n in names[1:4]] + [q0[4]]
                    if ok(q):
                        cost = max(abs(a - b) for a, b in zip(q[1:4], prev[1:4]))
                        if best is None or cost < best[0]:
                            best = (cost, q)
            if best is not None:
                break
        if best is None:
            break  # above here the line is out of reach: the 75 mm pose joins it at the highest point
        out.append(best[1])
        prev = best[1]
    if not out:
        return [q0], q0
    return out[::-1], q0


def dice_steps(current_slide=None, start=None):
    """--dice [N]: dice face DICE_FACE from the stored pucks (DICE_*), one timed move per step, ends at the nominal
    pose. Per puck: clock slot to the guide, the pick (pick_steps), the nominal via pose, 75 mm over its pip
    (tool_roll to 0), place, open, back up to 75 mm in one move."""
    start = start if start is not None else nominal_pose()
    via = via_pose()
    m = TaskCommand.MANIPULATOR
    pips = DICE_FACES[DICE_FACE]
    total = len(pips)
    s = [('dice 0: to the nominal via pose (gripper stays open)', m, via[:5])]
    for k, (row, col) in enumerate(pips, 1):
        lm, ff = dice_lm(col)
        pose = lambda h, row=row, lm=lm: [lm] + teach.raw_to_target([0] + DICE_ROWS[row][h]
                                                                    + [DICE_PLACE_ROLL_RAW_DEG, 0])[1:5]
        n = f'dice {k}/{total}'
        # The clock keeps whatever slot the last run left at the guide: the first slot is commanded too (arm up).
        s.append((f'{n} clock to slot {DICE_SLOTS[k - 1]} (stored puck {k} to the guide)', TaskCommand.CLOCK,
                  DICE_SLOTS[k - 1]))
        if k > 1:
            s.append((f'{n} to the nominal via pose (tool_roll back to 90)', m, via[:5]))
        s += pick_steps(f'{n} pick:')
        s += [
            (f'{n} to the carry via pose with the puck, shoulder out first (wrist folded, tool_roll 90)', m, carry_via_path()),
            (f'{n} 75 mm over the {row}-{col} pip (tool_roll to raw {DICE_PLACE_ROLL_RAW_DEG:g}{ff})', m, pose(75)),
            (f'{n} put down on the {row}-{col} pip, one move', m, pose(0)),
            (f'{n} open gripper full (release)', TaskCommand.GRIPPER, False),
            # One move (2026-11-07, the 40 mm stop dropped): TCP up to 7.8 mm off the straight line (near row), along
            # the column; the open jaws stay >= 20 mm from the neighbours 60 mm away.
            (f'{n} up to 75 mm, one move', m, pose(75)),
        ]
    s.append(('dice end: to the nominal via pose (gripper full open)', m, via[:5]))  # work -> via -> nominal (user 2026-11-13)
    s.append(('dice end: to the nominal pose (gripper full open)', m, start))
    # Shoulder I off while it waits at the nominal pose (2026-11-07, user): run sis_1107_1343 held the nominal pose
    # at 0.6 A on the shoulder for minutes and the shoulder got hot (48 C, the others 37-40) and weak. Same target, no
    # move: only the gains change, once it has arrived at 0.5 deg (with no I it may settle ~0.5 deg off, so this
    # step has the via tolerance). The next run's first step sends the base gains back.
    s.append(('dice end: hold the nominal pose, shoulder I off', m, start))
    return s


def unload_steps(current_slide=None, start=None):
    """sis2 (2026-11-06), the taught waypoints one move each (time-based profile: every joint of a move starts and
    arrives together), clock left where it is (slot 12 at the guide): nominal -> nominal via -> over the stored puck
    (tool_roll to 0, gripper part open) -> down to the grasp (two moves) -> close -> back up (two moves) -> nominal
    via -> over the place pose -> descend onto the box -> open -> tool_roll 90 deg -> ascend -> nominal (the place
    part only once PLACE_ABOVE_RAW_DEG is taught). Poses taught raw (jog, teach.yaml)."""
    start = start if start is not None else nominal_pose()
    via = via_pose()
    approach = teach.raw_to_target(PRE_GRASP_RAW_DEG)[:5]
    mid = teach.raw_to_target(PICK_MID_RAW_DEG + [0.0])[:5]
    over = teach.raw_to_target(PICK_ABOVE_RAW_DEG + [0.0])[:5]
    start_down = teach.raw_to_target(PICK_DESCEND_FROM_RAW_DEG + [0.0])[:5]
    place = teach.raw_to_target(PLACE_RAW_DEG + [0.0])[:5]
    above = None if PLACE_ABOVE_RAW_DEG is None else teach.raw_to_target(list(PLACE_ABOVE_RAW_DEG[:5]) + [0.0])[:5]
    roll_out = teach.raw_to_target([0, 0, 0, 0, PLACE_RELEASE_ROLL_RAW_DEG, 0])[4]
    m = TaskCommand.MANIPULATOR
    return [
        ('sis2 1: to the nominal via pose, one move (gripper stays open)', m, via[:5]),
        (f'sis2 2: over the stored puck (40 mm, wrist +1.5), tool_roll stays {PICK_ABOVE_RAW_DEG[4]:g}, gripper open to '
         f'{PICK_OPEN_GRIPPER_DEG:g} deg', m, start_down + [math.radians(PICK_OPEN_GRIPPER_DEG)]),
        ('sis2 4: down to the grasp, one move (no 20 mm stop)', m, approach),
        ('sis2 5: close gripper (grasp the stored puck)', TaskCommand.GRIPPER, True),
        ('sis2 6: up 20 mm (tool vertical)', m, mid),
        ('sis2 7: up to 40 mm over the slot', m, over),
        ('sis2 8: to the carry via pose with the puck, shoulder out first (wrist folded, tool_roll 90, gripper stays closed)', m, carry_via_path()),
    ] + ([] if PLACE_ABOVE_RAW_DEG is None else [
        ('sis2 9: over the place pose', m, above),
        ('sis2 10: descend onto the box (puck touches)', m, place),
        ('sis2 11: open gripper full (release the puck)', TaskCommand.GRIPPER, False),
        (f'sis2 12: tool_roll to raw {PLACE_RELEASE_ROLL_RAW_DEG:g} (90 deg turn, jaws open), the rest stays', m,
         place[:4] + [roll_out]),
        ('sis2 13: ascend over the place pose', m, above[:4] + [roll_out]),
        ('sis2 13b: to the nominal via pose (gripper full open)', m, via[:5]),  # back through the via (user 2026-11-13)
        ('sis2 14: to the nominal pose, one move (gripper full open)', m, start),
    ])



# bigbro: LM at the clock slot, relative to the sis2 slot poses [m], + = toward the LM motor. 2026-11-14 (user, bag
# sis_1114_0145): the front-picked cylinder came in ~5 mm off the slot mouth with the LM exactly on its target (7.19 mm,
# 0.01 mm error), so the slot poses move 3 mm away from the motor (user: ~3 mm would have gone in; first set to 5). sis2 / dice keep their taught LM. --slot-lm-mm N
# overrides it for one run.
BIGBRO_SLOT_LM = (float(sys.argv[sys.argv.index('--slot-lm-mm') + 1]) if '--slot-lm-mm' in sys.argv else -3.0) / 1000


BIGBRO_RELEASE_LOWER_MM = 18.0  # 10, then 8 more (user 2026-11-14, twice): release at sis2 grasp + 22 mm


def bigbro_steps(current_slide=None, start=None):
    """One bigbro_one per --plan file / --slot pair, in one run (2026-11-14, user: after loading a puck the arm goes
    straight on to the next one, not back to the nominal pose). All plans come from one upper-cam frame taken at the
    nominal pose (cam_pick.py --count N), so the arm never has to clear the camera in between. Between pucks: carry
    via -> via -> the next pick; only the last puck ends at the nominal pose. Labels get ' [puck k]' (step times are
    kept per label)."""
    global PLAN_FILE, BIGBRO_SLOT
    if len(PLAN_FILES) == 1:
        return bigbro_one(current_slide, start, True)
    if len(BIGBRO_SLOTS) != len(PLAN_FILES):
        sys.exit(f'--plan has {len(PLAN_FILES)} files, --slot {len(BIGBRO_SLOTS)} slots: give one slot per plan')
    s = []
    for k, (PLAN_FILE, BIGBRO_SLOT) in enumerate(zip(PLAN_FILES, BIGBRO_SLOTS)):
        one = bigbro_one(current_slide, start, k == len(PLAN_FILES) - 1)
        if k:  # already at the via pose (bigbro 8 of the puck before)
            one = [x for x in one if not x[0].startswith('bigbro 0:')]
        s += [(f'{n} [puck {k + 1}]', kind, a) for n, kind, a in one]
    return s


def bigbro_one(current_slide=None, start=None, home=True):
    """bigbro (2026-11-13, to compare with sis1): the sis1 pick of the puck in front (plan.yaml / --plan), then into
    clock slot 12 through the poses sis2 runs there (taught 2026-11-06 after the clock reassembly), not the sis1
    loading pose (hand-set 2026-11-03, 36 mm off slot 12 by FK). Independent of the nominal pose except the start and
    the end: via -> pick -> wrist folded -> clock 12 -> over the slot -> down to PICK_MID (sis2 grasp + 20 mm: the sis1
    grip holds the puck lower in the jaws than sis2 does) -> open to RELEASE_GRIPPER_DEG -> up -> shoulder out, carry
    via -> via -> nominal."""
    pick = steps_from_plan(current_slide, start)
    pick = pick[:[n for n, _, _ in pick].index('ascend') + 1]
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    over = teach.raw_to_target(PICK_DESCEND_FROM_RAW_DEG + [0.0])[:5]
    mid = teach.raw_to_target(PICK_MID_RAW_DEG + [0.0])[:5]
    over[0] += BIGBRO_SLOT_LM; mid[0] += BIGBRO_SLOT_LM
    # Release height: PICK_MID is the sis2 grasp + 20 mm, fine for a 45 mm grip. A shallower grip leaves the puck that
    # much lower under the jaws, so the release goes up by the difference (toward the 40 mm over pose, joint blend).
    a = yaml.safe_load(open(HERE / PLAN_FILE)).get('assume', {})
    grip_mm = 1000 * a.get('puck_height', 0.06) * a.get('grasp_coverage', 0.75)
    # Release height over the sis2 grasp [mm]: 22 for a 20 mm grip (2026-11-14, user, tuned on the robot: 40, then 30,
    # then 22). A deeper grip holds the puck's bottom that much higher in the jaws, so the release goes that much lower
    # (prism, 30 mm grip: 12). Between PICK_MID (+20) and the over pose (+40) by f, below PICK_MID extended (f < 0).
    # 2026-11-14 13:25: the grips are set to the depths the loaded pucks really had (cylinder 34, prism 38), and the
    # release heights that went in with them are kept as they were: cylinder +22, prism +12.
    above_mm = 12.0 if grip_mm >= 36.0 else 40.0 - BIGBRO_RELEASE_LOWER_MM
    f = min(max((above_mm - 20.0) / 20.0, -0.5), 1.0)
    release = [m_ + (o_ - m_) * f for m_, o_ in zip(mid, over)]
    lift = pick[-1][2][-1]  # the ascend pose
    # A puck with a neighbour nearer than the full-open jaws allow: cam_pick.py gives a smaller opening (open_deg).
    try:
        import json
        narrow = next(p for p in json.load(open(HERE / 'plan_cam_pucks.json')) if p['plan'] == PLAN_FILE).get('open_deg')
    except (OSError, StopIteration, ValueError):
        narrow = None
    if narrow:
        n0, k0, a0 = pick[0]  # move 1: [[LM, shoulder, elbow, wrist_pitch, tool_roll, gripper]]
        pick[0] = (n0.replace('gripper full open', f'gripper open to {narrow:g} deg (a puck close by)'), k0,
                   [a0[0][:5] + [math.radians(narrow)]])
    # In: fold the wrist to the slot angle first (as sis1 load 1, TCP up to 501 mm), then straight over the slot
    # (420 mm). Through the carry via instead the TCP rises to 570 mm (FK), past where a puck hit the gripper rest.
    fold = lift[:3] + [over[3]] + lift[4:5]
    s = [('bigbro 0: to the nominal via pose', m, via)]
    pick = [(f'bigbro pick: {n}', k, a) for n, k, a in pick]
    over_puck = [n for n, _, _ in pick].index('bigbro pick: move 2: shoulder above the puck') + 1
    pick.insert(over_puck, ('bigbro check: gripper cam sees the puck lined up under the jaws', GRIPCHECK, None))
    s += pick
    s += [('bigbro 1: wrist_pitch folded to the slot angle first, the rest stays', m, fold),
          (f'bigbro 2: clock to slot {BIGBRO_SLOT} (loading guide)', TaskCommand.CLOCK, BIGBRO_SLOT),
          (f'bigbro 3: over clock slot {BIGBRO_SLOT} (sis2 over pose, 40 mm)', m, over),
          (f'bigbro 4: down into the slot to sis2 grasp + {20 + 20 * f:.0f} mm (grip {grip_mm:.0f} mm)', m, release),
          (f'bigbro 5: open gripper to {RELEASE_GRIPPER_DEG:g} deg (release in the slot)', TaskCommand.GRIPPER,
           math.radians(RELEASE_GRIPPER_DEG)),
          ('bigbro 6: up to 40 mm over the slot', m, over),
          ('bigbro 7: to the carry via pose, shoulder out first', m, carry_via_path()),
          ('bigbro 8: to the nominal via pose', m, via)]
    if start is not None and home and '--no-home' not in sys.argv:  # --no-home: stay at the via pose, the next puck starts there
        s.append(('bigbro 9: to the nominal pose', m, start))
    return s

def gripcal_steps(current_slide=None, start=None):
    """--gripcal (pa22, 2026-11-14): camera axes of the gripper cam in the arm frame, for the bigbro correction. With a
    puck in front and its plan (--plan plan_cam.yaml): via -> over the puck (40 mm, as bigbro) -> shot -> GRIPCAL_MM
    sideways (+LM axis) -> shot -> back and GRIPCAL_MM ahead -> shot -> back over the puck -> grip_axes.json -> via
    -> nominal. The puck is not touched; it must not move during the run."""
    pick = steps_from_plan(current_slide, start)
    pick = pick[:[n for n, _, _ in pick].index('move 2: shoulder above the puck') + 1]
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    approach = pick[-1][2][-1]
    if abs(math.degrees(yaml.safe_load(open(HERE / PLAN_FILE))['assume'].get('tool_roll', 0.0))) > GRIPFIX_ROLL_DEG:
        sys.exit('gripcal: this plan turns the tool (a turned prism): use a cylinder, the axes are measured at tool_roll 0')
    side, ahead = replan(GRIPCAL_MM, 0.0, 'plan_gripcal_side.yaml'), replan(0.0, GRIPCAL_MM, 'plan_gripcal_ahead.yaml')
    if side is None or ahead is None:
        sys.exit('gripcal: plan.py has no pose 8 mm beside / ahead of this puck: put the puck nearer the middle')
    s = [('bigbro 0: to the nominal via pose', m, via)]
    s += [(f'bigbro pick: {n}', k, a) for n, k, a in pick]
    s += [('bigbro gripcal: shot over the puck', GRIPCAL, 'c0'),
          (f'bigbro gripcal: arm {GRIPCAL_MM:g} mm sideways (+LM axis)', m, [side[0]]),
          ('bigbro gripcal: shot', GRIPCAL, 'c1'),
          (f'bigbro gripcal: arm back, then {GRIPCAL_MM:g} mm ahead', m, [approach, ahead[0]]),
          ('bigbro gripcal: shot', GRIPCAL, 'c2'),
          ('bigbro gripcal: arm back over the puck', m, [approach]),
          ('bigbro gripcal: camera axes', GRIPCAL, 'solve'),
          ('bigbro 8: to the nominal via pose', m, via)]
    if start is not None and '--no-home' not in sys.argv:
        s.append(('bigbro 9: to the nominal pose', m, start))
    return s


class Runner(Node):
    def __init__(self):
        super().__init__('puck_pick_execute')
        self.cmd = self.create_publisher(TaskCommand, '/planning/task_command', 10)
        self.hold = self.create_publisher(Bool, '/planning/hold', 10)
        self.result, self.status, self.grasped, self.joints = None, None, None, None
        self.create_subscription(JointState, '/control/arm_joints', lambda m: setattr(self, 'joints', m), 10)
        self.create_subscription(TaskResult, '/control/task_result', lambda m: setattr(self, 'result', m), 10)
        self.create_subscription(ControlStatus, '/control/status', lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Bool, '/control/gripper_grasped', lambda m: setattr(self, 'grasped', m.data), 10)
        self.hold_value = False
        self.create_timer(0.1, lambda: self.hold.publish(Bool(data=self.hold_value)))  # 10 Hz heartbeat
        self.task_id = int(time.time())  # monotonic across runs; control rejects retired (smaller) IDs

    def send(self, kind, arg, seconds=0.0, tolerance=0.0, i_gains=None, contact=0.0):
        self.task_id += 1
        c = TaskCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.task_id, c.revision, c.operation, c.kind = self.task_id, 1, TaskCommand.EXECUTE, kind
        if kind == TaskCommand.MANIPULATOR:
            c.duration_s = float(seconds)  # travel time (time-based profile); 0 = the speed caps
            c.tolerance_rad = float(tolerance)  # arrival tolerance [motor rad]; 0 = control.yaml (0.5 deg)
            c.position_i_gains = [int(g) for g in (i_gains or [])]  # shoulder..tool_roll; empty = power-up gains
            c.speed = float(contact)  # unused by the arm: > 0 = stop this move on contact (control ContactDetector)
        if kind == TaskCommand.MANIPULATOR:  # a sixth value: open the gripper to that motor angle [rad] meanwhile
            c.joints.name, c.joints.position = list(ARM) + ['gripper'] * (len(arg) - 5), [float(v) for v in arg]
        elif kind == TaskCommand.GRIPPER:  # True close, False full open, a float: open to that motor angle [rad]
            c.close = arg is True
            if not isinstance(arg, bool):
                c.joints.name, c.joints.position = ['gripper'], [float(arg)]
        elif kind == TaskCommand.CLOCK:
            c.clock_slot = int(arg)
        self.result = None
        self.cmd.publish(c)
        return self.task_id

    def wait(self, task_id, timeout=35.0):
        t = time.time()
        while time.time() - t < timeout:
            r = self.result
            if r is not None and r.task_id == task_id and r.status not in (TaskResult.RUNNING, TaskResult.STOPPING):
                return STATUS.get(r.status, str(r.status)), r.reason
            time.sleep(0.05)
        return 'TIMEOUT', 'no terminal result'

    def stop(self):
        c = TaskCommand(); c.operation = TaskCommand.STOP; c.task_id = 0
        self.cmd.publish(c); self.hold_value = True


def describe(kind, arg):
    if kind == TaskCommand.MANIPULATOR:  # a path of poses after stage()
        return [round(v, 4) for v in arg[-1]] + ([f'{len(arg)} moves'] if len(arg) > 1 else [])
    if kind == TaskCommand.CLOCK:
        return f'slot {arg}'
    if kind == GRIPCHECK:
        return (f'look, correct the arm if > {GRIPFIX_MM:g} mm off, stop if > {GRIPFIX_MAX_MM:g}' if GRIP_AXES.exists()
                else f'look, stop if > {GRIPCHECK_MM:g} mm off')
    if kind == GRIPZERO:
        return 'gripper-cam shot -> grip_zero.json'
    if kind == GRIPCAL:
        return 'solve and write grip_axes.json' if arg == 'solve' else f'gripper-cam shot {arg}'
    if not isinstance(arg, bool):
        return f'open to {math.degrees(arg):.1f} deg'
    return 'close' if arg else 'open'


# auto, choi or --auto anywhere on the command line (dice 6 auto): no Enter between the steps.
AUTO = any(a in sys.argv[1:] for a in ('auto', 'choi', '--auto'))


# Arrival tolerance of a MANIPULATOR step (2026-11-07): moves to the (nominal) via pose need no precision, so they
# count as arrived within 3 motor degrees per joint (control and the arm board, TaskCommand.tolerance_rad) and the
# next move starts without waiting out the slow last-degree creep (bag sis_1107_0324: 2-4 s per via). Everything else
# (the grasp, the place, the nominal pose) keeps control.yaml's 0.5 deg.
# 2026-11-07 (user): in the place, only the putting down needs precision, so going back up after the release takes
# VIA_TOLERANCE_DEG too (sis_1107_0444: 8.0 -> 2.9 s over 3 pucks; the clock and the via pose come next). The swing to
# 75 mm over the pip keeps 0.5 deg: tried at 3 deg in 0444, the put-down then started while the joints were still
# swinging from it and pushed the puck 1.2-3.3 mm into the box for 4-6 s (0438 at 0.5 deg: within +-1.1 mm).
VIA_TOLERANCE_DEG = 3.0
LOOSE_STEPS = ('via pose', 'up to 75 mm, one move', 'shoulder I off')


def step_tolerance(name):
    """Arrival tolerance [motor rad] of a MANIPULATOR step: VIA_TOLERANCE_DEG for LOOSE_STEPS, 0 = control.yaml."""
    return math.radians(VIA_TOLERANCE_DEG) if any(k in name for k in LOOSE_STEPS) else 0.0


# Position I gain per step (2026-11-07 experiment; firmware IGAIN, TaskCommand.position_i_gains): shoulder, elbow,
# wrist_pitch, tool_roll. I_GAINS_BASE = firmware lm_arm config.h kPositionIGains (keep equal); a step whose label
# contains a STEP_I_GAINS key gets those gains instead. One small change at a time, one run + bag each.
# Run 1 (sis_1107_0425): shoulder 100 on every move that lifts the TCP. Lifting the puck out of the slot got better
# (overshoot 1.15 -> 0.18 deg, 2.5 -> 0.6 s), but going up after the release (0.9 -> 4.2 s) and over the stored puck
# (2.5 -> 3.4 s, 2.7 -> 4.0 s) crept slower: there the shoulder comes in short, not past.
# Run 2 (from bags 0405 + 0425, 6 moves per step and joint): per step, a joint that mostly goes past its goal and
# creeps back (> 0.5 deg past, > 1 s tail) gets about 20 % less I; one that comes in short or is fine keeps its I.
#   over the stored puck: elbow past 4/6 (tail 4.6 s), wrist past 4/6 (2.0 s); shoulder short 4/6 -> keeps 130
#   lift out of the slot: shoulder 100 (run 1)
#   75 mm over the pip: elbow past 3/6 (up to 5.2 s), wrist and tool_roll past 6/6 (1.9 / 1.5 s)
#   put down: elbow past 2/6 (4.1 s), wrist past 3/6; shoulder short 2/6 -> keeps 130
#   up to 75 after the release: elbow past 2/6, wrist past 4/6; shoulder mixed (past 4.2 deg / short) -> keeps 130
#   to the nominal pose: elbow, wrist, tool_roll past 2/2 (3.8 / 1.7 / 2.2 s)
#   via poses: arrive within 3 deg anyway -> base
# Run 2 result (sis_1107_0438, 36.6 s/puck, best so far): less I helped the short vertical moves (put down: elbow tail
# 3.3 -> 0.5 s, up to 75: elbow 4.0 -> 0.5 s, shoulder past 4.19 -> 0.67 deg; the release jumps less: open 14.0 -> 8.3 s)
# but not the big swings (75 mm over the pip 20.7 -> 28.8 s, over the stored puck 14.2 -> 15.3, to the nominal pose
# 6.3 -> 7.3): there the joints go just as far past the goal (not I wind-up) and only come back slower.
# Run 3: run 2's gains for the put down and up to 75 only; the big swings back to base.
I_GAINS_BASE = [130, 100, 200, 100]
STEP_I_GAINS = {
    'pick: up to 40 mm over the slot': [100, 100, 200, 100],
    'put down on the': [130, 80, 160, 100],
    'up to 75 mm, one move': [130, 80, 160, 100],
    'shoulder I off': [0, 100, 200, 100],  # dice end, waiting at the nominal pose
}


def step_i_gains(name):
    """[shoulder, elbow, wrist_pitch, tool_roll] Position I gains of a MANIPULATOR step (STEP_I_GAINS, else base)."""
    return list(next((g for key, g in STEP_I_GAINS.items() if key in name), I_GAINS_BASE))


# Stop on contact (2026-11-07, user): the put-down ends where the puck meets the box instead of pressing on to its
# taught height (sis_1107_1343/1451: the far puck met the box 10-18 mm over the target and the jaws hit the box).
# control_master (ContactDetector): shoulder current under this ratio of its mid-move level, or the joints stalled
# late in the move; the arm's target becomes where it is and the step succeeds with reason "contact (...)".
# Ratio 0.7 (was 0.4; sis_1107_1513: at 0.4 the far put-down went 7-10 mm on after the current began to fall, it
# falls over ~0.2 s). Replayed on 0455/1237/1246/1343/1451/1513: 0.7 x2 no false hit, far hits 0.08-0.16 s sooner
# (1513: 1.72 -> 1.64 s, 1.66 -> 1.50 s); 0.8 and 0.7 x1 hit good runs.
STEP_CONTACT = {}  # rolled back to 1d6226a (user 2026-11-14): no contact stop on the dice put down (was {'put down on the': 0.7})


def step_contact(name):
    """Stop-on-contact ratio of a MANIPULATOR step (STEP_CONTACT), 0 = off; sent as TaskCommand.speed."""
    return next((r for key, r in STEP_CONTACT.items() if key in name), 0.0)


def step_seconds(name, k=0):
    """Travel time [s] of a MANIPULATOR step (the first STEP_SECONDS key in its label), 0 = as fast as the caps
    allow."""
    v = SLOW_SECONDS.get(name)
    if v:  # one time per pose of the step (main move, then the slow final approach)
        return v[min(k, len(v) - 1)]
    return next((s for key, s in STEP_SECONDS.items() if key in name), 0.0)


DESCEND_PASS_DEG = 1.0  # [deg] arrival at the passing poses (a piece moves a joint only ~4-9 deg: 3 deg would cut it short)
DESCEND_PIECE_S = 1.0  # [s] each pose of a several-pose 'pick: descend' (3 poses: 3 s for ~55 mm; one pose keeps 2 s)


def seconds_text(name):
    v = SLOW_SECONDS.get(name)
    return ' + '.join(f'{x:.1f}' for x in v) + ' s' if v else (f'{step_seconds(name):g} s' if step_seconds(name) else '')


# Start check: the LM this far from the nominal pose means the arm was put down elsewhere (--lm-tol-mm N widens it
# for one run, e.g. after a power cut left the carriage 10 mm off with its turn count read right).
LM_START_TOL = float(sys.argv[sys.argv.index('--lm-tol-mm') + 1]) / 1000 if '--lm-tol-mm' in sys.argv else 0.010
# Carousel memory (2026-11-14, user: the task loads 12 pucks, two per stop, and must remember what sits in every
# slot). carousel.yaml: slot -> class (red/green + cylinder/prism, from cam_pick.py's plan_cam_pucks.json). A slot is
# written when its release step (bigbro 5) succeeded. carousel.py shows / resets it and gives the next free slots.
CAROUSEL = HERE / 'carousel.yaml'


def record_loaded(name, grasped):
    """After a succeeded 'bigbro 5' (release in the slot): write that puck's class into its slot in carousel.yaml."""
    import json
    import re
    k = int(m.group(1)) - 1 if (m := re.search(r'\[puck (\d+)\]', name)) else 0
    plan_file, slot = PLAN_FILES[k], BIGBRO_SLOTS[k]
    try:
        info = next(p for p in json.load(open(HERE / 'plan_cam_pucks.json')) if p['plan'] == plan_file)
    except (OSError, StopIteration, ValueError):
        info = {'class': 'unknown'}  # a plan not from cam_pick.py (plan.yaml)
    data = (yaml.safe_load(open(CAROUSEL)) if CAROUSEL.exists() else None) or {}
    data[slot] = {'class': info['class'], 'score': info.get('score'), 'time': time.strftime('%m-%d %H:%M:%S'),
                  'grasped': bool(grasped)}
    yaml.safe_dump(data, open(CAROUSEL, 'w'), sort_keys=True)
    print(f'   carousel: slot {slot} = {info["class"]}' + ('' if grasped else '  (!! gripper did not report a grasp: check the slot)'))


GRIPCHECK = 'gripcheck'   # pseudo step kind: gripper-cam look at the puck before going down (bigbro)
HOLD_BEFORE_CLOSE_S = float(sys.argv[sys.argv.index('--hold-before-close') + 1]) if '--hold-before-close' in sys.argv else 0.0
LOOK_STEPS = ('bigbro pick: move 1', 'bigbro pick: move 2', 'bigbro fix')  # sent with tool_roll 0 (run_steps)
GRIPCHECK_MM = 10.0       # stop when the puck sits further than this from the lined-up reference (camera x / y)
LEARN_REF = None         # capture prefix of a check passed without a reference, stored as the reference after a grasp
GRIP_LOCATE = Path.home() / 'craft' / 'tools' / 'realsense_check' / 'grip_locate.py'


# Gripper-cam auto correction (pa22 patch gripcam_autofix_20261114, merged by hand into the 2026-11-14 flow; NOT run on
# the robot before the merge). --gripcal measures how the image of a still puck shifts per arm mm (grip_axes.json);
# with that file a puck further than GRIPFIX_MM off the lined-up reference moves the arm (plan.py again for the puck
# shifted by the error) and the camera looks again, up to GRIPFIX_TRIES times; further than GRIPFIX_MAX_MM stops.
# Without the file: the 10 mm check as before. Off for a turned tool (prism, |tool_roll| > GRIPFIX_ROLL_DEG): the
# axes were measured at tool_roll 0 and how the camera turns with the roll is not known.
GRIP_AXES = Path.home() / 'craft' / 'tools' / 'realsense_check' / 'grip_axes.json'  # written by --gripcal
GRIPFIX_MM = 3.0
NO_GRIPFIX = '--no-gripfix' in sys.argv  # the gripper cam only reads and prints: no arm correction and no stop for an offset
                                          # (2026-11-14, user: the first approach was good, the corrections moved the jaws onto the puck)
MAX_REACH = 0.150  # [m] reach with the gripper-cam correction (18:45: at 156 mm the shoulder could not lift the arm, 1.1 A, and the jaws pressed the puck top)
GRIPFIX_MAX_MM = 35.0  # was 20: the first look after --gripzero read 29 mm (camera), the arm 24.5 mm short
GRIPFIX_TRIES = 3
GRIPFIX_DZ_MM = 4.0       # camera-to-top distance off the reference by more: the height is corrected first
GRIP_Z_PER_SIDE = 0.375   # [camera mm per arm mm sideways] (--gripcal frames: 8 mm sideways, camera z +3 mm; ahead 0)
GRIP_DZ_TO_UP = 1.08      # camera distance -> vertical (the camera looks ~22 deg off vertical, same frames)
GRIPFIX_ROLL_DEG = 3.0
GRIPCAL = 'gripcal'       # pseudo step kind of --gripcal: gripper-cam shots around known arm moves
GRIPCAL_MM = 8.0          # size of the two calibration moves (sideways along the LM axis, then ahead)


GRIP_TURN_SIGN = +1.0       # tool_roll [URDF] = sign * (tag turn in the picture - zero). From --gripcal: picture x = -base x,
                            # picture y = +base y, a mirror, so picture turn = -yaw, and tool_roll = -yaw (FK). One check
                            # on the robot: upper cam yaw -0.9 deg read +0.6 in the picture. NOT yet seen on a turned prism.
GRIP_TURN_ZERO_DEG = 0.0
GRIP_TURN_MIN_DEG = 6.0     # smaller differences are left alone (the gripper-cam yaw is within ~4 deg of the upper cam's)


def held_bad(name, shot, steps, i):
    """'' when the closed jaws hold the puck as a well-held one (grip_locate.py --held on the closed-jaw frame), else the
    reason. Turned tools are not checked: the reference is a square-held puck and a turned one looks wider."""
    import re
    import subprocess
    if not HELD_CHECK:
        return ''
    q = next((steps[j][2][-1] for j in range(i, len(steps)) if 'pick: ascend' in steps[j][0]), None)
    if q is not None and abs(math.degrees(q[4])) > 10.0:
        print('   held check skipped: the jaws are turned (no reference for that yet)')
        return ''
    p = GRIP_LOCATE.parent / 'out' / shot.replace('saved ', '')
    r = subprocess.run(['/usr/bin/python3', str(GRIP_LOCATE), '--held', str(p), '--shape', 'prism' if is_prism(name) else 'cylinder'],
                       capture_output=True, text=True, timeout=60)
    line = next((l for l in r.stdout.splitlines() if 'HELD' in l), 'HELD UNKNOWN no answer')
    print('   ' + line.split(': ', 1)[-1])
    return 'held wrong (' + line.split('HELD BAD ', 1)[1] + ')' if 'HELD BAD' in line else ''


HELD_CHECK = '--no-held-check' not in sys.argv


def is_prism(name):
    """True when the step's puck is a prism (cam_pick.py's plan_cam_pucks.json)."""
    import json
    try:
        return 'prism' in next(p for p in json.load(open(HERE / 'plan_cam_pucks.json')) if p['plan'] == step_plan(name))['class']
    except (OSError, StopIteration, ValueError):
        return False


def step_plan(name):
    """The plan file of the puck a step belongs to (' [puck k]' in its label, else the only one)."""
    import re
    m = re.search(r'\[puck (\d+)\]', name)
    return PLAN_FILES[int(m.group(1)) - 1] if m else PLAN_FILES[0]


def grip_measure():
    """One gripper-cam frame (tools/realsense_check/grip_locate.py, the puck under the jaws). Returns (cam xyz [mm] or
    None, offset from the lined-up reference (dx, dy) [mm] or None, the printed line, the capture prefix or None)."""
    import re
    import subprocess
    r = subprocess.run(['/usr/bin/python3', str(GRIP_LOCATE), '--capture', '--under-jaws'], capture_output=True, text=True, timeout=60)
    text = r.stdout + r.stderr
    line = next((l for l in text.splitlines() if 'camera x' in l or 'no puck' in l), text.strip()[-200:])
    num = r'([+-]?\d+(?:\.\d+)?)'
    c = re.search(rf'camera x {num} y {num} z {num} mm', text)
    m = re.search(rf'dx {num} dy {num} dz {num} mm', text)
    seen = re.search(r'^(\S+): \w+ [\d.]+ px', text, re.M)
    cam = None if not c else [float(c.group(i)) for i in (1, 2, 3)]
    off = None if not m else [float(m.group(1)), float(m.group(2))]
    grip_check.dz = None if not m else float(m.group(3))
    t = re.search(rf'tag x {num} y {num} z {num}', text)
    grip_measure.tag = None if not t else [float(t.group(i)) for i in (1, 2, 3)]
    tt = re.search(rf'arm yaw {num} deg', text)
    grip_measure.turn = float(tt.group(1)) if tt else None  # the top's yaw in the arm frame (not the picture angle)
    return cam, off, line, seen.group(1) if seen else None


def grip_axes(name=''):
    """2x2 matrix M of --gripcal: gripper-cam shift of a still puck (dx, dy) [mm] per arm move (sideways +LM axis,
    ahead) [mm]. None: not calibrated. Holds at any tool_roll: the camera sits before the roll joint (2026-11-14:
    the tool rolled 45 deg, the picture stayed and the jaws turned in it)."""
    import json
    if not GRIP_AXES.exists():
        return None
    return np.array(json.load(open(GRIP_AXES))['cam_per_arm'], dtype=float)


def grip_check(name=''):
    """Gripper-cam look at the puck against its lined-up reference. Returns (ok, reason, off): off = (dx, dy) [mm] in
    the camera frame, None when nothing was compared. With grip_axes.json the limit is GRIPFIX_MM (run_steps corrects
    the arm), without it GRIPCHECK_MM."""
    global LEARN_REF
    LEARN_REF = None
    cam, off, line, prefix = grip_measure()
    print('   ' + line)
    zero = GRIP_LOCATE.parent / 'grip_zero.json'  # per shape since 12:15; a shape without one reads 'no reference' below
    prev = getattr(grip_check, 'prev', None)
    if (zero.exists() and cam is not None and off is None and grip_measure.tag is None and prev is not None
            and prev[0] == step_plan(name) and 'again after the correction' in name):
        # After a correction the turned jaw can hide a corner of the top tag (2026-11-14: a prism at -41 deg, the tag
        # in plain view but not decoded, its tape gives no depth). The puck has not moved, only the camera: the tag
        # moved in the picture as the box centre did. Estimated from the last look at this puck.
        import json
        zs = json.load(open(zero))
        sh = 'prism' if is_prism(name) else 'cylinder'
        if sh in zs and zs[sh].get('point') == 'tag':
            est = np.array(prev[2]) + (np.array(cam) - np.array(prev[1]))
            d3 = est - np.array(zs[sh]['mm'])
            if abs(d3[2]) <= 30.0:
                off = [float(d3[0]), float(d3[1])]
                grip_check.dz = float(d3[2])
                print(f'   top tag hidden: estimated from the last look (box centre moved {cam[0] - prev[1][0]:+.1f} / '
                      f'{cam[1] - prev[1][1]:+.1f} mm) -> vs jaw centre dx {off[0]:+.1f} dy {off[1]:+.1f} mm')
    for _ in range(3):  # the jaws' centre is known but this frame's top tag was not read: look again
        if not (zero.exists() and cam is not None and off is None):
            break
        cam, off, line, prefix = grip_measure()
        print('   ' + line)
    if cam is not None and grip_measure.tag is not None:
        grip_check.prev = (step_plan(name), list(cam), list(grip_measure.tag))
    if cam is None:
        return False, 'gripper cam: no puck', None
    if off is None and zero.exists():
        return False, 'gripper cam: the top tag was not read in 4 frames (hidden by a jaw? glare?): not going down blind', None
    if off is None:
        # The puck is seen but there is no reference at this height (new grip depth, first run). Not a stop
        # (2026-11-14, user: no hand calibration, handle it inside the run): go down on the upper-cam plan, and when the
        # gripper then reports a grasp this frame becomes the reference for its shape (run_steps).
        LEARN_REF = prefix
        return True, 'gripper cam sees the puck; no reference at this height yet, this frame becomes it if the grasp holds', None
    lim = GRIPFIX_MM if grip_axes(name) is not None else GRIPCHECK_MM
    dx, dy = off
    # Height: the camera-to-top distance against the reference's. The camera also comes nearer as the arm goes
    # sideways (GRIP_Z_PER_SIDE), so the part the arm's own offset explains is taken out. The upper cam reads the shelf
    # 6-17 mm low (2026-11-14: dz -5..-16 mm on every prism look), so the approach stood lower than at the reference and
    # the x / y read there were skewed by the slant view (13:08: an 11.6 mm 'offset' moved the arm 14.6 mm sideways).
    grip_check.dz_h = 0.0
    M = grip_axes(name)
    if grip_check.dz is not None and M is not None:
        d_side = -np.linalg.solve(M, np.array(off))[0]
        grip_check.dz_h = grip_check.dz + GRIP_Z_PER_SIDE * d_side
        if abs(grip_check.dz_h) > GRIPFIX_DZ_MM:
            return False, f'gripper cam: the puck top is {-grip_check.dz_h * GRIP_DZ_TO_UP:+.1f} mm off the planned height (camera {grip_check.dz:+.1f} mm)', off
    if max(abs(dx), abs(dy)) > lim:
        return False, f'gripper cam: puck {dx:+.1f} / {dy:+.1f} mm off the lined-up spot (limit {lim:g})', off
    return True, f'gripper cam: puck {dx:+.1f} / {dy:+.1f} mm off, within {lim:g}', off


def replan(d_side_mm, d_ahead_mm, out='plan_cam_corr.yaml', plan_file=None, roll=None, d_shelf_mm=0.0):
    """plan.py again for the puck of plan_file (default PLAN_FILE) moved by (sideways along the LM axis, ahead) [mm];
    same shelf height, grip depth and tool_roll. Returns (approach, grasp) arm targets like plan_pose, or None when
    plan.py finds no pose."""
    import subprocess
    a = yaml.safe_load(open(HERE / (plan_file or PLAN_FILE)))['assume']
    # The correction must not carry the arm past the reach the motors hold (cam_pick.py MAX_AHEAD; 2026-11-14 12:17:
    # a +18.9 mm correction took a puck at the limit to ~158 mm and the jaws knocked it over on the way down).
    reach = a['puck_ahead'] + d_ahead_mm / 1000
    if reach > MAX_REACH:
        print(f'   the corrected puck is {reach * 1000:.0f} mm ahead of the LiDAR face (> {MAX_REACH * 1000:.0f}): no correction; park closer')
        return None
    r = subprocess.run([sys.executable, str(HERE / 'plan.py'), '--puck-ahead', f"{a['puck_ahead'] + d_ahead_mm / 1000:.4f}",
                        '--puck-x', f"{a['puck_x_offset'] + d_side_mm / 1000:.4f}", '--shelf-height', f"{a['shelf_height'] + d_shelf_mm / 1000:.4f}",
                        '--grip-mm', f"{a['grasp_coverage'] * a['puck_height'] * 1000:.1f}",
                        '--tool-roll', f"{a.get('tool_roll', 0.0) if roll is None else roll:.4f}",
                        '--approach-above', f"{a.get('approach_above', 0.04):.3f}", '--out', out],
                       capture_output=True, text=True)
    f = yaml.safe_load(open(HERE / out)).get('frames') if r.returncode == 0 and (HERE / out).exists() else None
    if not f:
        print('   plan.py found no pose for the corrected puck:\n   ' + '\n   '.join((r.stdout + r.stderr).strip().splitlines()[-3:]))
        return None
    pose = plan_pose(out)
    return pose(0), descent_path(pose, out)  # approach, the way down (its last pose = the grasp)


GRIPZERO = 'gripzero'     # pseudo step kind of --gripzero


def gripzero_step(what):
    """--gripzero: one gripper-cam frame stored as the jaws' centre (grip_locate.py --set-zero)."""
    import subprocess
    if what == 'look':  # a plain colour frame of the closed jaws on the puck, to look at afterwards
        p = GRIP_LOCATE.parent / 'out' / time.strftime('gripclose_%H%M%S')
        r = subprocess.run([str(GRIP_LOCATE.parent / 'build' / 'rs_snap'), str(p), '1280', '720', '261622073363'], capture_output=True, text=True, timeout=60)
        return r.returncode == 0, f'saved {p.name}'
    r = subprocess.run(['/usr/bin/python3', str(GRIP_LOCATE), '--capture', '--under-jaws', '--set-zero'], capture_output=True, text=True, timeout=60)
    line = next((l for l in (r.stdout + r.stderr).splitlines() if 'zero' in l), (r.stdout + r.stderr).strip()[-200:])
    return 'zero stored' in line, line


def gripzero_steps(current_slide=None, start=None):
    """--gripzero (2026-11-14): where a puck sits in the gripper-cam picture when it is in the middle of the jaws,
    found without a hand: over the puck (as bigbro) -> down -> close (the jaws push the puck to their middle, front-back)
    -> open -> up to the approach pose -> a shot: its top tag's centre is the jaws' centre for every later check and
    correction (grip_zero.json). Replaces the references learned from a grasp, which kept the error of that grasp
    (cylinders came in off the middle and were knocked away). Sideways the jaws do not centre the puck: that direction
    stays as the upper cam put it. Use a standing puck with a top tag, not turned (a prism within a few degrees)."""
    pick = steps_from_plan(current_slide, start)
    names = [n for n, _, _ in pick]
    pick = pick[:names.index('ascend') + 1]
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    i = [n for n, _, _ in pick].index('ascend')
    pick.insert(i, ('open gripper (the puck stays where the jaws centred it)', TaskCommand.GRIPPER, False))
    pick.insert(i, ('gripzero: shot of the closed jaws', GRIPZERO, 'look'))
    s = [('bigbro 0: to the nominal via pose', m, via)]
    s += [(f'bigbro pick: {n}', k, a) for n, k, a in pick]
    s += [('bigbro gripzero: shot of the centred puck', GRIPZERO, 'shot'),
          ('bigbro 8: to the nominal via pose', m, via)]
    if start is not None:
        s.append(('bigbro 9: to the nominal pose', m, start))
    return s


_gripcal = {}


def gripcal_step(what):
    """--gripcal: store one shot ('c0', 'c1', 'c2') or solve and write grip_axes.json ('solve')."""
    import json
    if what != 'solve':
        cam, _, line, _ = grip_measure()
        print('   ' + line)
        if cam is None:
            return False, 'gripper cam: no puck'
        _gripcal[what] = np.array(cam[:2])
        return True, f'shot {what}: camera x {cam[0]:+.1f} y {cam[1]:+.1f} mm'
    c0, c1, c2 = (_gripcal.get(k) for k in ('c0', 'c1', 'c2'))
    if c0 is None or c1 is None or c2 is None:
        return False, 'a shot is missing'
    M = np.column_stack([(c1 - c0) / GRIPCAL_MM, (c2 - c0) / GRIPCAL_MM])
    n1, n2 = np.linalg.norm(M[:, 0]), np.linalg.norm(M[:, 1])
    cosang = float(M[:, 0] @ M[:, 1] / max(n1 * n2, 1e-9))
    print(f'   camera shift per arm mm: sideways {np.round(M[:, 0], 3)} (|{n1:.2f}|), ahead {np.round(M[:, 1], 3)} (|{n2:.2f}|),'
          f' angle between them {math.degrees(math.acos(max(-1, min(1, cosang)))):.0f} deg')
    # A still puck seen from a camera that moves 8 mm must shift about 8 mm, and the two directions are about square.
    if not (0.6 < n1 < 1.4 and 0.6 < n2 < 1.4 and abs(cosang) < 0.35):
        return False, 'the shifts do not look like 8 mm moves at right angles: nothing written (puck moved? wrong puck detected?)'
    json.dump({'cam_per_arm': M.tolist(), 'move_mm': GRIPCAL_MM, 'stamp': time.strftime('%Y-%m-%d %H:%M'),
               'plan': str(PLAN_FILE)}, open(GRIP_AXES, 'w'), indent=1)
    return True, f'written: {GRIP_AXES}'


def run_steps(node, steps):
    """Run steps one Enter at a time ('bro' runs the rest without asking; 'choi' / --auto on the command line: all of
    them from the start); False if the user quit or a step did not succeed."""
    auto = AUTO
    grasped_at_close = None
    fixed, tries, shelf_fix = np.zeros(2), 0, 0.0  # this puck's arm correction so far (sideways, ahead, up) [mm], corrections done
    roll_fix = None  # this puck's tool_roll after a gripper-cam turn correction [rad]
    for i, (name, kind, arg) in enumerate(steps):  # GRIPCHECK may insert steps after itself
        if auto:
            print(f'>> {name}')
        else:
            answer = input(f'>> {name}? ').strip().lower()
            if answer == 'q':
                return False
            auto = answer == 'bro'
        if name.startswith('bigbro pick: close') and HOLD_BEFORE_CLOSE_S > 0:
            # --hold-before-close N: the jaws stay open at the grasp pose N s (a photo from the side: is the tool upright?)
            print(f'   holding {HOLD_BEFORE_CLOSE_S:g} s at the grasp pose with the jaws open (take the photo now)')
            gripzero_step('look')
            time.sleep(HOLD_BEFORE_CLOSE_S)
        if kind in (GRIPCAL, GRIPZERO):
            ok, reason = gripcal_step(arg) if kind == GRIPCAL else gripzero_step(arg)
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            continue
        if kind == GRIPCHECK:
            if 'again after the correction' not in name:  # the first look at a puck: its corrections start at zero
                fixed, tries, shelf_fix = np.zeros(2), 0, 0.0
            if 'again after the correction' not in name:
                roll_fix = None
            ok, reason, off = grip_check(name)
            if NO_GRIPFIX and reason != 'gripper cam: no puck':
                if is_prism(name) and grip_measure.turn is None and 'again after the correction' not in name:
                    for k in range(2):  # a prism's top tag was not read: shoot again (the jaws stay at roll 0 without it)
                        line = grip_measure()[2]
                        print(f'   top tag not read, shot {k + 2}: {line}')
                        if grip_measure.turn is not None:
                            break
                    if grip_measure.turn is None:
                        reason += '; PRISM YAW NOT READ in 3 shots, the jaws are not turned'
                ok, reason = True, 'NOT CORRECTED (--no-gripfix): ' + reason  # a prism's jaw turn below still runs
            M = grip_axes(name)
            # A prism: the jaws turn to its faces as the gripper cam sees them from 20 cm (2026-11-14: the upper cam
            # reads the top tag's turn only now and then). The camera does not turn with the tool, so the tag's turn
            # in the picture is the tool_roll that lines the jaws up (GRIP_TURN_SIGN, GRIP_TURN_ZERO_DEG).
            turn_fix = None
            if is_prism(name) and grip_measure.turn is not None:
                want = -math.radians(grip_measure.turn)  # tool_roll = -yaw (FK; as cam_pick.py)
                have = roll_fix if roll_fix is not None else yaml.safe_load(open(HERE / step_plan(name)))['assume'].get('tool_roll', 0.0)
                d_roll = (want - have + math.pi / 4) % (math.pi / 2) - math.pi / 4
                if abs(d_roll) > math.radians(GRIP_TURN_MIN_DEG):
                    turn_fix = have + d_roll
            need_move = not NO_GRIPFIX and not ok and off is not None and M is not None and max(abs(off[0]), abs(off[1])) <= GRIPFIX_MAX_MM
            if (need_move or (turn_fix is not None and (ok or need_move))) and tries < GRIPFIX_TRIES:
                # The puck is off the lined-up spot by `off` in the camera. A still puck shifts by M @ (arm move), so
                # the arm move that brings it back is -inv(M) @ off. The pick is planned again for the puck there.
                d = -np.linalg.solve(M, np.array(off)) if need_move else np.zeros(2)
                d_up = 0.0
                if need_move and abs(getattr(grip_check, 'dz_h', 0.0)) > GRIPFIX_DZ_MM:
                    # height first; the x / y are looked at again from the reference height
                    d_up, d = -grip_check.dz_h * GRIP_DZ_TO_UP, np.zeros(2)
                    shelf_fix += d_up
                if turn_fix is not None:
                    roll_fix = turn_fix
                    reason += f'; prism yaw {grip_measure.turn:+.1f} deg (arm frame) -> tool_roll {math.degrees(roll_fix):+.1f} deg'
                poses = replan(*(fixed + d), plan_file=step_plan(name), roll=roll_fix, d_shelf_mm=shelf_fix)
                if poses is None:
                    print(f'   -> FAILED {reason}; no pose for the correction')
                    print('step did not succeed')
                    return False
                if poses is not None:
                    fixed, tries = fixed + d, tries + 1
                    approach2, grasp2 = poses
                    if shelf_fix:
                        # The box is fixed: the height correction only puts the camera where its reference was taken
                        # (x / y read right there); the grasp keeps the fixed shelf's depth (13:26: raised 7 mm, the
                        # prism was held 7 mm shallow and the held check stopped it).
                        g0 = replan(*(fixed), plan_file=step_plan(name), roll=roll_fix, out='plan_cam_corr0.yaml')
                        if g0 is not None:
                            grasp2 = g0[1]
                    tag = name[name.index(' [puck'):] if ' [puck' in name else ''
                    for j in range(i + 1, len(steps)):  # this puck's descend and ascend only
                        n, k2, _ = steps[j]
                        if 'pick: descend' in n:
                            steps[j] = (n, k2, grasp2)
                        elif 'pick: ascend' in n:
                            steps[j] = (n, k2, [approach2]); break
                    steps.insert(i + 1, (f'bigbro fix {tries}: over the puck, arm {d[0]:+.1f} mm sideways {d[1]:+.1f} mm ahead {d_up:+.1f} mm up'
                                         f' (total {fixed[0]:+.1f} / {fixed[1]:+.1f}){tag}', TaskCommand.MANIPULATOR, [approach2]))
                    steps.insert(i + 2, (f'bigbro check: gripper cam again after the correction{tag}', GRIPCHECK, None))
                    print(f'   -> CORRECTING {reason}')
                    continue
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            if name.startswith('bigbro check'):
                # The jaws turn to the puck now, over it (the descend pose holds the same roll).
                asc = next((j for j in range(i + 1, len(steps)) if 'pick: ascend' in steps[j][0]), None)
                if asc is not None:
                    q = steps[asc][2][-1]
                    if abs(q[4]) > math.radians(1.0):
                        tag = name[name.index(' [puck'):] if ' [puck' in name else ''
                        steps.insert(i + 1, (f'bigbro pick: turn the jaws to the puck, tool_roll {math.degrees(q[4]):+.1f} deg{tag}',
                                             TaskCommand.MANIPULATOR, [q]))
            continue
        if kind == TaskCommand.MANIPULATOR and name.startswith(LOOK_STEPS):
            # Looked at with the jaws square to the camera: turned they hide the top tag and the detector loses the
            # puck (2026-11-14, prism at 43 deg). The roll turns about the tool axis, so the position is the same;
            # the jaws turn to the puck after the last look (TURN step below).
            arg = [list(q[:4]) + [0.0] + list(q[5:]) for q in arg]
        for k, target in enumerate(arg if kind == TaskCommand.MANIPULATOR else [arg]):
            gains = None
            if kind == TaskCommand.MANIPULATOR:
                gains = step_i_gains(name)
                if gains != I_GAINS_BASE:
                    print(f'   I gains {"/".join(map(str, gains))} (STEP_I_GAINS)')
            secs, tol = step_seconds(name, k), step_tolerance(name)
            if kind == TaskCommand.MANIPULATOR and 'the vertical line' in name and k < len(arg) - 1:
                tol = math.radians(VIA_TOLERANCE_DEG)  # dice: passing poses on the line, no waiting for 0.5 deg
            if kind == TaskCommand.MANIPULATOR and 'pick: descend' in name and len(arg) > 1:
                secs = DESCEND_PIECE_S
                if k < len(arg) - 1:  # passing poses on the way down: no waiting for 0.5 deg; the grasp keeps it
                    tol = math.radians(DESCEND_PASS_DEG)
            status, reason = node.wait(node.send(kind, target, secs, tol, gains,
                                                 step_contact(name) if kind == TaskCommand.MANIPULATOR else 0.0))
            if status != 'SUCCEEDED':
                break
        extra = f'  grasped={node.grasped}' if kind == TaskCommand.GRIPPER and arg is True else ''
        print(f'   -> {status} {reason}{extra}')
        if status != 'SUCCEEDED':
            print('step did not succeed')
            if kind == TaskCommand.MANIPULATOR and node.joints is not None:
                # A joint (LM: 0.5 motor deg = 0.06 mm) left more than the 0.5-degree tolerance away (e.g. sagging under load) keeps the
                # board "moving", and control completes nothing until it settles.
                now = dict(zip(node.joints.name, node.joints.position))
                for n, t in zip(ARM, target):
                    err = now[n] - t
                    unit = f'{err * 1000:+.1f} mm' if n == 'slide_joint' else f'{math.degrees(err):+.1f} deg'
                    flag = '  <-- off' if (abs(err) > 0.00006 if n == 'slide_joint' else abs(err) > math.radians(0.5)) else ''
                    print(f'     {n:18s} target {t:+.4f} actual {now[n]:+.4f}  error {unit}{flag}')
            return False
        if name.startswith('bigbro 5:'):
            record_loaded(name, grasped_at_close)
        if kind == TaskCommand.GRIPPER and arg is True:
            grasped_at_close = node.grasped
            if name.startswith('bigbro pick: close'):
                # A colour frame of the closed jaws, kept for looking at afterwards (2026-11-14: pucks were not held and
                # nothing showed why), and no trip to the clock with empty jaws: without a grasp the run stops here.
                ok_shot, shot = gripzero_step('look')
                print(f'   gripper cam: {shot}')
                bad = held_bad(name, shot, steps, i)
                if bad:
                    # Held wrong (sideways along the jaws, too shallow, knocked over): it would not go into the slot.
                    # Let go over the box and stop; basic, then the next run looks again.
                    print(f'   -> {bad}: letting it go and stopping before the clock')
                    node.wait(node.send(TaskCommand.GRIPPER, False))
                    asc = next((steps[j][2][-1] for j in range(i + 1, len(steps)) if 'pick: ascend' in steps[j][0]), None)
                    if asc is not None:
                        node.wait(node.send(TaskCommand.MANIPULATOR, asc, 0.0, 0.0, I_GAINS_BASE, 0.0))
                    return False
                if not node.grasped:
                    print('   -> the gripper reports no grasp: not carrying on to the clock (basic brings the arm back)')
                    return False
            if LEARN_REF and node.grasped:
                import subprocess
                r = subprocess.run(['/usr/bin/python3', str(GRIP_LOCATE), '--under-jaws', '--set-ref', LEARN_REF], capture_output=True, text=True)
                print('   gripper cam: ' + (r.stdout.strip().splitlines() or ['reference not stored'])[-1])
    return True


def go_home(node, start, lying):
    """After a failed step or q: rise above the puck where the LM is (lying: up its vertical line to LYING_HIGH
    first when the tool is out there), then LM and arm back to the start, elbow first."""
    time.sleep(0.5)
    s = node.status
    if s is not None and s.fault:
        print(f'control FAULT ("{s.reason}"): the board latched a fault and accepts nothing until it reboots.\n'
              'Support the arm, then resetarm; the LM turn count is lost, so put the carriage at the nominal pose\n'
              'before the board starts again (same as a power-up).')
        node.stop()
        return
    if input('\nreturn to the start pose step by step? Enter = yes, n = no: ').strip().lower() == 'n':
        node.stop()
        return
    now = dict(zip(node.joints.name, node.joints.position))
    slide = min(max(now['slide_joint'], -0.0381), 0.0627)
    rise = [plan_pose()(20)[1:]]
    if lying:
        g = yaml.safe_load(open(HERE / 'plan_lying.yaml'))['puck_grasp_urdf']
        tcp = plan.Chain(plan.URDF).fk(now)[:3, 3]
        if tcp[1] < (g[1] + plan.ASSUME['lidar_front_y']) / 2:  # tool out over the lying puck
            h0 = max(tcp[2] - g[2], 0) + STAGE_SEG / 1000
            up = lying_line(np.arange(h0, LYING_HIGH + 1e-6, STAGE_SEG / 1000))  # 1 mm per plan frame
            rise = up + tcp_path(up[-1], rise[0], LYING_SWING)
    steps = [('open gripper (drops a held puck)', TaskCommand.GRIPPER, False),
             ('rise above the puck height, LM stays', TaskCommand.MANIPULATOR, [[slide] + r for r in rise]),
             ('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + rise[-1]),
             ('arm back to the start pose', TaskCommand.MANIPULATOR, start)]
    steps = stage(with_via(steps, start, out=False), [now[n] for n in ARM])
    if not run_steps(node, steps):
        print('return stopped; sending STOP + hold'); node.stop()


def main():
    dry = '--dry-run' in sys.argv
    lying = '--lying' in sys.argv
    dice = '--dice' in sys.argv
    bigbro = '--bigbro' in sys.argv
    gripcal = '--gripcal' in sys.argv
    gripaxis = '--gripzero' in sys.argv
    unload = '--unload' in sys.argv or dice or bigbro or gripcal or gripaxis  # these set their own way out and stop at their own end
    make_steps = (gripzero_steps if gripaxis else gripcal_steps if gripcal else bigbro_steps if bigbro else dice_steps if dice else unload_steps if unload
                  else lying_steps_from_plan if lying else steps_from_plan)
    goto = sys.argv[sys.argv.index('--goto') + 1] if '--goto' in sys.argv else None
    if goto is not None:
        if goto not in ('nominal', 'via', 'basic'):
            print('--goto nominal|via|basic'); return 1
        make_steps = lambda *a: ([('to the nominal pose', TaskCommand.MANIPULATOR, nominal_pose())]
                                 if goto == 'nominal' else out_to_via(nominal_pose()) + (
                                     [('to the nominal pose', TaskCommand.MANIPULATOR, nominal_pose())] if goto == 'basic' else []))
    steps = make_steps() if make_steps is steps_from_plan else stage(make_steps(), None)
    for name, kind, arg in steps:
        print(f'  {name:45s}', describe(kind, arg), seconds_text(name))
    if dry:
        return 0
    rclpy.init()
    node = Runner()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        t = time.time()
        while node.status is None and time.time() - t < 3:
            time.sleep(0.1)
        if node.status is None:
            print('no /control/status: is control_master running?'); return 1
        time.sleep(1.0)  # heartbeat must be alive before control accepts tasks
        s = node.status
        print(f'control: ready={s.ready} fault={s.fault} held={s.held} reason="{s.reason}"')
        if not s.ready or s.fault:
            print('control is not ready; aborting'); return 1
        if node.joints is None:
            print('no /control/arm_joints; aborting'); return 1
        now = dict(zip(node.joints.name, node.joints.position))
        slide = now['slide_joint']
        print(f'LM now: slide {slide:+.4f} m (nominal pose {nominal_pose()[0]:+.4f})')
        if not -0.040 <= slide <= 0.064:
            print('LM position is outside the calibrated range (wrong start position?). Run checkarm; aborting'); return 1
        slide = min(max(slide, -0.0381), 0.0627)
        limits = plan.Chain(plan.URDF).limits
        outside = [n for n in ARM[1:] if not limits(n)[0] <= now[n] <= limits(n)[1]]
        if outside:  # control clamps these into the limits, so the first move pulls them in
            print(f'note: arm joints outside the limits {outside}; control pulls them inside on the first move.')
        # Every run starts from and returns to the nominal pose.
        start = nominal_pose()
        # Wherever it was put down, the first move takes it to the nominal pose; a joint 90 deg off (LM 10 mm)
        # means it was put down somewhere else entirely (e.g. tool_roll a half turn away).
        # via / nom / basic bring the arm back from wherever a stopped run left it (2026-11-14: a bigbro stopped over
        # the puck left the LM 10.6 mm off and basic refused): any LM inside the calibrated range checked above is
        # accepted there. A work flow still has to start at the nominal pose.
        lm_tol = LM_START_TOL if goto is None or '--lm-tol-mm' in sys.argv else 1.0
        # The same for the arm joints: over a puck the wrist_pitch is 160 deg and the tool_roll 90 deg from the nominal
        # pose (2026-11-14: basic refused after a stopped gripaxis). With every joint inside its limits the way back
        # (out_to_via: shoulder, then elbow / LM / roll, then wrist) is the reverse of the way out.
        joint_tol = math.radians(90.0) if goto is None or outside else math.radians(400.0)
        off = [f'{n} {(now[n] - t) * 1000:+.1f} mm' if n == 'slide_joint' else f'{n} {math.degrees(now[n] - t):+.1f} deg'
               for n, t in zip(ARM, start) if abs(now[n] - t) > (lm_tol if n == 'slide_joint' else joint_tol)]
        if off:
            print(f'!! too far from the nominal pose ({", ".join(off)}): move it closer by hand; aborting'); return 1
        steps = make_steps(via_pose()[0], start)
        if goto is None:
            if not unload:  # sis2 sets its own way out and stops at the front
                steps = with_via(steps, start)  # the work starts with the LM at the via pose
            elif all(abs(now[n] - v) < (0.001 if n == 'slide_joint' else math.radians(0.5))
                     for n, v in zip(ARM, via_pose())):
                print('already at the via pose: the move out to it is skipped')
                steps = [s for s in steps if not s[0].startswith(('sis2 1:', 'dice 0:', 'bigbro 0:'))]
        elif goto == 'nominal':  # wrist_pitch before the elbow
            if not close_but_shoulder([now[n] for n in ARM], start):
                steps = [wrist_first([now[n] for n in ARM], start, 'the nominal pose')] + steps
        else:  # from where it was put down: shoulder, elbow, wrist_pitch
            steps = out_to_via([now[n] for n in ARM])
            if goto == 'basic':  # via, then nom, in one run (2026-11-14, user): the via pose, then into the nominal pose
                via = via_pose()
                if not close_but_shoulder(via, start):
                    steps.append(wrist_first(via, start, 'the nominal pose'))
                steps.append(('to the nominal pose', TaskCommand.MANIPULATOR, start))
        if make_steps is not steps_from_plan:  # sis1 sets its own move order
            steps = stage(steps, [slide] + [now[n] for n in ARM[1:]])
        # Moves to / from the via and nominal pose: slow and eased (time-based), not at the speed caps (a 40 degree
        # shoulder move at 69 deg/s dropped the arm, 2026-11-13, user). Same in --goto and every flow.
        prev = [slide] + [now[n] for n in ARM[1:]]
        new_steps = []
        for name, kind, arg in steps:
            if kind != TaskCommand.MANIPULATOR:
                new_steps.append((name, kind, arg)); continue
            # Smooth, time-based: the way into the nominal pose (with the slow final approach) and every move to a via
            # pose (one move each, no split): at the speed caps the via moves jerked the motors (user 2026-11-13).
            # Work steps keep their STEP_SECONDS.
            into_nominal = ('nominal pose' in name and 'via pose' not in name) or (
                start is not None and all(abs(a - b) < 1e-6 for a, b in zip(arg[-1][:5], start[:5])))  # e.g. sis1 'back 3'
            if not (into_nominal or 'via pose' in name):
                prev = arg[-1]; new_steps.append((name, kind, arg)); continue
            out, times = [], []
            for j, q in enumerate(arg):
                deg = max(abs(q[i] - prev[i]) for i in range(1, 5)) * 180 / math.pi
                if into_nominal and j == len(arg) - 1 and deg > 2 * APPROACH_DEG:  # the last APPROACH_DEG of the farthest joint: own slow move
                    f = APPROACH_DEG / deg
                    out.append([q[i] - (q[i] - prev[i]) * f if i < len(prev) else q[i] for i in range(len(q))])
                    times.append(max(GOTO_MIN_S, (deg - APPROACH_DEG) / GOTO_DEG_S))
                    out.append(q); times.append(max(APPROACH_MIN_S, APPROACH_DEG / APPROACH_DEG_S))
                else:
                    out.append(q); times.append(max(GOTO_MIN_S, deg / (GOTO_DEG_S if into_nominal else VIA_DEG_S)))
                prev = q
            SLOW_SECONDS[name] = times
            new_steps.append((name, kind, out))
        steps = new_steps
        for name, kind, arg in steps:
            print(f'  {name:45s}', describe(kind, arg), seconds_text(name) if kind == TaskCommand.MANIPULATOR else '')
        if AUTO:
            print('\nAUTO (choi): every step runs without asking. Keep a hand on the 12 V power; Ctrl+C = STOP + hold.')
            for k in (3, 2, 1) if goto is None else ():  # via / nom / basic start at once (2026-11-14, user)
                print(f'  starting in {k} s ...'); time.sleep(1)
        else:
            print('\nKeep a hand on the 12 V power. Enter = run step, q + Enter = quit.\n')
        # Control must have been out of hold for a moment before the first task: a task sent while the heartbeat was
        # still settling was accepted and stopped at once (2026-11-14, basic without the 3 s countdown: STOPPED, the
        # shoulder never moved). Wait for held = False for 0.5 s in a row, 5 s at most.
        others = node.count_publishers('/planning/hold') - 1
        if others > 0:
            print(f'!! {others} other program(s) publish /planning/hold (an earlier puckload / bigbro still open in another '
                  'terminal?): Ctrl+C it first; nothing sent'); return 1
        t0, free_since = time.time(), None
        while time.time() - t0 < 5.0:
            free_since = (free_since or time.time()) if not node.status.held else None
            if free_since and time.time() - free_since >= 0.5:
                break
            time.sleep(0.05)
        else:
            print('control stays held (hold heartbeat / a stop not finished): nothing sent'); return 1
        if not run_steps(node, steps):
            if goto is None and not unload:
                go_home(node, start, lying)
            else:
                print('sending STOP + hold'); node.stop()
        if goto is not None:  # via / nom: done, control (and the board) keep holding the pose
            print('done: control keeps holding the pose.')
            return 0
        # The script ends here (2026-11-14: a stopped bigbro left waiting for Ctrl+C kept publishing hold = True, and
        # every later basic / via was stopped at once). Control holds the pose by itself once the heartbeat is gone.
        print('done: control keeps holding the last pose.')
        return 0
    except KeyboardInterrupt:
        print('\nCtrl+C: sending STOP + hold'); node.stop(); time.sleep(0.5)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code or 0)  # not sys.exit: the spin thread aborted the interpreter on the way out ("terminate called", core dumped)
