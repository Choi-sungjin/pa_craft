#!/usr/bin/env python3
"""Throwaway demo: pick the planned puck on the REAL robot and load it into the clock, via control_master.
--unload (sis2): nominal -> via -> over the stored puck -> down -> close -> up -> via -> place on the box -> nominal,
one timed move each (taught poses, clock not turned).
--dice [N]: dice face N (1-6, default 6) from the stored pucks, one per clock slot (DICE_*), back to the nominal pose.
--stand: standing-puck pre-descend check: via -> over each of the 9 dice grid cells (75 mm, the dice ascend pose) -> via.
--stand --cell N: only cell N (1-9, stand order), and it stays there (back home: --goto basic).
--lie --cell N --lie-ref: the lying puck put where lie_grasp takes it: the shot at the wait pose becomes the cell's reference.
--lie --cell N [--pick]: a lying prism at cell 2 or 5: gripper cam long axis -> jaws turned -> pick -> put down at 8, long axis along x.
--planning --box NAME: route steps through planning_master; lying wheel pauses become 0.28 m reverse / re-dock.
--collect: fresh two-puck survey, standing first then nearest; dispatch standing/lying pick.
--find: a standing puck anywhere between cells 8 and 2: upper cam -> wait pose -> gripper cam fix -> pick -> load.
--standcal --cell N (2, 5, 8): calibration for the upper-cam -> gripper-cam correction, writes stand_cal.json.
--goto cell --cell N: from where the arm is now (e.g. over another cell) straight to cell N's pick wait pose, one move.
--yaw (with --stand --cell N or --goto cell): then the gripper cam reads the prism's yaw and the jaws turn to it (tool_roll).
--goto nominal|via: one move to that pose (elbow first, shoulder last), nothing else.
--lying picks the lying puck of plan_lying.yaml (plan.py --lying) on its long axis and loads it the same way.

Sends craft_interfaces/TaskCommand on /planning/task_command one step at a time and waits for
/control/task_result. Keeps the /planning/hold heartbeat alive (control rejects tasks without it).
Press Enter before every step; Ctrl+C sends STOP and hold.
Usage:  source ~/craft/install/setup.bash && python3 ~/craft/demo/puck_pick/execute.py [--unload | --dice | --lying | --goto nominal|via] [--dry-run]
Delete with: rm -rf ~/craft/demo
"""
import json
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
from craft_perception.msg import PuckObservationArray
from puck_observations import upper_observations, grip_observations
from pick_policy import candidates as policy_candidates, select as policy_select
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Bool, String

import plan  # URDF joint limits
import fold_motion
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
STAND_VIA_DEG_S = 40.0  # standing pick / load via moves (user 2026-11-15 "too slow": lift -> carry via swings the wrist 206 deg, 10.3 s at 20)
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
    'bigbro pick: descend': 2.0,
    'stand pick: descend': 2.0,  # one move 75 mm -> grasp, time-based like the dice put-down (user 2026-11-15: one move)  # per pose of descent_path when it has several: see step_seconds
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
# left), p14 (4, middle left), p15 (7, near left), p16 (9, near right); the rest filled in (teach.yaml p17 = 6, p18 = 3; renamed 11-15 to dice_place_c<cell>_*, cells 1-3 = far row):
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
              'put down', 'rise above the puck', 'sis2 ', 'dice ', 'bigbro ', 'stand ', 'standcal ', 'standfind ', 'lie ', 'lie regrasp',
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


def dice_line(lm, row, q0_raw=None):
    """2026-11-14 (user: the tool must stay square to the box while it puts a puck down). Poses from the 75 mm pose
    down the vertical line over the pip to the place pose: the TCP keeps the place pose's x / y and the tool keeps
    its angle (2.2 deg off vertical) wherever the joints allow (margin 3 deg); higher up it leans the least that fits
    and straightens on the way down. Model (URDF): near row square from 55 mm, middle from 45, far only the last 5 mm
    (arm near full reach). Returns (way down: line top .. DICE_FINAL_DROP, the final pose), each [slide, s, e, w, roll]."""
    ch = plan.Chain(plan.URDF)
    names = plan.Chain.NAMES
    q0 = [lm] + teach.raw_to_target([0] + (q0_raw or DICE_ROWS[row][0]) + [DICE_PLACE_ROLL_RAW_DEG, 0])[1:5]
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


STAND_ROLL_RAW_DEG = 90.0  # pick wait pose: jaws 90 deg turned from the dice put-down (raw 0) (user 2026-11-15)


def stand_yaw_tail():
    """--yaw: at the wait pose the gripper cam reads the prism's yaw; the jaws' turn is inserted when it is known."""
    return [('stand yaw: gripper cam reads the prism yaw', STANDYAW, None)] if '--yaw' in sys.argv else []


# --pick (user 2026-11-15 "후딱 집고 후딱 적재"): grasp pose = the 2026-11-14 taught dice place poses (jog-taught with a
# puck held from the clock, its bottom on the box: the jaws sit at the clock grip depth on a standing puck; the
# DICE_ROWS place poses are 15 mm over these). Down in one move (user 11-15), close, up to 75 mm,
# clock to the slot (arm over the box), carry via, then the bigbro load: over the slot, release STAND_RELEASE_MM over
# the sis2 grasp (the same grip depth as sis2), open, up, carry via, via, nominal.
STAND_GRASP_ROWS = {'far': [112.8, 238.9, -73.5], 'middle': [96.9, 267.4, -61.5], 'near': [84.4, 283.8, -59.0]}
STAND_RELEASE_MM = 10.0


STAND_BASE = {}  # pick step name -> its poses before jaw_fix (stand_yaw_step redoes the fix at the turned roll)


def jaw_fix(q, off_m=None):
    """q [slide, shoulder, elbow, wrist_pitch, tool_roll] -> the same with the TCP moved by minus the bent jaws' offset
    at q's roll (STAND_JAW_OFF_M toward the LM motor at the nominal roll, turned with the roll), tool angle kept."""
    off_m = STAND_JAW_OFF_M if off_m is None else off_m
    ch = plan.Chain(plan.URDF)
    names = plan.Chain.NAMES
    fk = lambda q: ch.fk(dict(zip(names, [q[0] - LM_BACKLASH] + list(q[1:5]))))
    q = list(q[:5])
    s_hat = fk([q[0] + 0.001] + q[1:])[:3, 3] - fk(q)[:3, 3]
    s_hat /= np.linalg.norm(s_hat)  # +slide = toward the LM motor
    R_nom = fk(q[:4] + [nominal_pose()[4]])[:3, :3]
    v = fk(q)[:3, :3] @ (R_nom.T @ (s_hat * off_m))  # where the jaws take the puck, from the TCP [m]
    p = fk(q)[:3, 3]
    # the pose's tool angle in ik_top_down terms (as dice_line), then the same angle at p - v
    best = None
    for t in (x / 10 for x in range(-400, 401)):
        for sol in plan.ik_top_down(ch, p, t):
            c = max(abs(sol[n] - q[i + 1]) for i, n in enumerate(names[1:4]))
            if best is None or c < best[0]:
                best = (c, t)
    if best is None or best[0] > math.radians(0.5):
        raise RuntimeError(f'jaw_fix: no IK match for the pose ({best})')
    sols = plan.ik_top_down(ch, p - v, best[1])
    if not sols:
        raise RuntimeError('jaw_fix: the shifted pose is out of reach')
    sol = min(sols, key=lambda d: max(abs(d[n] - q[i + 1]) for i, n in enumerate(names[1:4])))
    return [float(sol['slide_joint'] + LM_BACKLASH)] + [float(sol[n]) for n in names[1:4]] + [q[4]]


# Grasp poses taught per cell (user 2026-11-15, teacharm from the computed grasp: the computed one had the tool curled in
# toward the robot; the LM also had to move for the bent jaws). One pose per cell: at another yaw only tool_roll
# changes (user). Cell -> teach.yaml name.
STAND_GRASP_TAUGHT = {2: 'stand_grasp_c2', 5: 'stand_grasp_c5', 8: 'stand_grasp_c8'}  # c2 taught at yaw ~26, c5 / c8 at yaw 0: only the roll differs


# All grasp poses 3 mm toward robot +y (= away from the LM motor = -slide), by the LM (user 2026-11-15: a slight twist).
# Trims from the user's eye after --find runs (2026-11-15: the gripper cam cannot read the puck at the grasp pose,
# too near / the jaws in the way): how far the GRIPPER should go. LM: STAND_GRASP_Y_FIX_M (- = toward the LM motor).
# Ahead: STAND_GRASP_AHEAD_FIX_MM (+ = away from the robot), grasp and approach poses of --find.
STAND_GRASP_AHEAD_FIX_MM = -10.0  # cells 1 and 2 (2026-11-15 18:4x): the grasp was 10 mm farther from the robot than the puck
# x fix by where the puck is (distance ahead after the gripper-cam fix, mm -> fix mm), from the user's eye
# (TCP - puck, x = robot front): linear between the points, the end values outside. Replaces STAND_GRASP_AHEAD_FIX_MM.
# Runs (fix used -> TCP - puck x -> fix that would have been right; x + = away from the robot):
# 18:39 c1 157.5 mm: 0 -> +10 -> -10; 18:42 c2 155.7: 0 -> +10 -> -10; 18:46 c2 159.0: -10 -> 0 -> -10;
# 18:48 c5 105.2: -10 -> -5 -> -5 (sign first given the other way); 18:50 c5 91.0: -15 -> -10 -> -5;
# 18:53 c5 94.1: -5 -> -3 -> -2.
STAND_X_FIX_TABLE = [
    (42.1, -10.0),   # cell 8 19:09: at -2 TCP 8 mm farther from the robot -> -10
    (97.0, -2.0),    # cell 5: the last run (-5) was 3 mm toward the robot -> -2 (user: each report is vs the last setting)
    (157.0, -10.0),  # cells 1 / 2 runs (155.7-159.0)
]


# Approach pose only (the pose the descent starts from), x + = away from the robot, by the puck's distance ahead.
# 18:59 cell 8 (approach 5 mm too far, caught the puck) was a misplaced puck (user): back to 0.
STAND_APPROACH_X_FIX_TABLE = [
    (52.5, 0.0),
    (97.0, 0.0),
]


def stand_approach_x_fix(ahead):
    t = sorted(STAND_APPROACH_X_FIX_TABLE)
    if ahead <= t[0][0]:
        return t[0][1]
    for (a0, f0), (a1, f1) in zip(t, t[1:]):
        if ahead <= a1:
            return f0 + (f1 - f0) * (ahead - a0) / (a1 - a0)
    return t[-1][1]


def _shift_try(q, d_mm):
    q = list(q[:5]); tgt = _plane_state(q) + np.array([-d_mm / 1000, 0.0, 0.0])
    for _ in range(80):
        e = tgt - _plane_state(q)
        if np.linalg.norm(e[:2]) < 2e-5 and abs(e[2]) < 2e-4:
            break
        J = np.zeros((3, 3))
        for j in range(3):
            dq = list(q); dq[j + 1] += 1e-5
            J[:, j] = (_plane_state(dq) - _plane_state(q)) / 1e-5
        d = np.clip(np.linalg.lstsq(J, e, rcond=None)[0], -0.05, 0.05)
        q = [q[0]] + [q[i + 1] + d[i] for i in range(3)] + [q[4]]
    e = tgt - _plane_state(q)
    ch = plan.Chain(plan.URDF)
    ok = (np.linalg.norm(e[:2]) < 3e-4 and abs(e[2]) < math.radians(0.3)
          and all(ch.limits(n)[0] <= q[i + 1] <= ch.limits(n)[1] for i, n in enumerate(plan.Chain.NAMES[1:4])))
    return ok, [float(v) for v in q]


def shift_ahead(q, d_mm):
    """q with the TCP moved d_mm away from the robot (- = toward it), height and tool angle kept (URDF model, small
    local move; the arm plane's -y is away from the robot). Near the model's stretched-out end (far row) it may not
    reach: then as far as it does, in 0.5 mm steps (user 2026-11-15: as much as possible)."""
    if abs(d_mm) < 0.05:
        return list(q), 0.0
    k = 1.0
    while k > 0:
        ok, r = _shift_try(q, d_mm * k)
        if ok:
            return r, d_mm * k
        k -= 0.5 / abs(d_mm)
    return list(q), 0.0


# y fix by the puck's distance ahead, on top of STAND_GRASP_Y_FIX_M, mm, + = +y = away from the LM motor (grasp and
# approach of --find). 19:09 cell 8: TCP 9 mm nearer the LM motor -> +9 there; cells 1 / 2 / 5 were right.
STAND_Y_FIX_TABLE = [
    (42.1, 4.0),     # 19:1x cell 8: at +9 TCP 5 mm farther from the LM motor -> +4
    (97.0, 0.0),
]


def _table(t, ahead):
    t = sorted(t)
    if ahead <= t[0][0]:
        return t[0][1]
    for (a0, f0), (a1, f1) in zip(t, t[1:]):
        if ahead <= a1:
            return f0 + (f1 - f0) * (ahead - a0) / (a1 - a0)
    return t[-1][1]


def stand_y_fix(ahead):
    return _table(STAND_Y_FIX_TABLE, ahead)


# y fix by where the LM is (the wait LM after the gripper-cam fix, mm), + = away from the LM motor. The other tables
# come from the centre column (LM ~12 mm). 20:33 (green prism, LM -40 = the far rail end, yaw +37): 3 mm short toward
# the LM motor -> -5 there (user: x right, y 5 too far; one run; the yaw 37 was also new).
STAND_LM_Y_FIX_TABLE = [
    (-40.0, -2.4),   # 20:36: TCP 1.5 toward the LM motor, but the LM stopped 3.9 short at the rail end -> the target was 2.4 too far
    (12.2, 0.0),
]


def stand_lm_y_fix(lm_mm):
    return _table(STAND_LM_Y_FIX_TABLE, lm_mm)


# y fix by the jaws' turn (the yaw the roll followed, signed), mm, + = away from the LM motor, per distance ahead
# (linear between the rows). Cell 5 at yaw +26-27 (19:42 / 19:48 / 19:50) sat toward the LM motor; 0 at yaw 0.
STAND_YAW_Y_FIX_TABLE = {
    97.0: [
        (-26.0, 1.0),  # 19:57 (yaw -31): at +4 the grasp should move 3 toward the LM motor -> +1
        (0.0, 0.0),
        (26.0, 4.0),   # 19:52 (yaw 27): at +3 still 2 short -> +5; 19:5x: 1 back toward the LM motor -> +4
    ],
    157.0: [
        (-26.0, 1.0),  # not run yet: as cell 5
        (0.0, 0.0),
        (26.0, 2.0),   # 20:06 cell 2 (yaw +36): at +4, 2 toward the LM motor -> +2
    ],
}


def stand_yaw_y_fix(yaw, ahead=97.0):
    pts = sorted((a, _table(t, yaw)) for a, t in STAND_YAW_Y_FIX_TABLE.items())
    return _table(pts, ahead)


# x fix by the jaws' turn (signed yaw), mm, + = away from the robot (on top of STAND_X_FIX_TABLE), cell 5.
STAND_YAW_X_FIX_TABLE = [
    (-26.0, 6.0),  # 19:57 (yaw -31): at +3 the grasp should move 3 farther from the robot -> +6
    (0.0, 0.0),
    (26.0, 3.0),   # 19:52 (yaw 27, cell 5): +3
]


def stand_yaw_x_fix(yaw):
    return _table(STAND_YAW_X_FIX_TABLE, yaw)


def stand_x_fix(ahead):
    t = sorted(STAND_X_FIX_TABLE)
    if ahead <= t[0][0]:
        return t[0][1]
    for (a0, f0), (a1, f1) in zip(t, t[1:]):
        if ahead <= a1:
            return f0 + (f1 - f0) * (ahead - a0) / (a1 - a0)
    return t[-1][1]
STAND_GRASP_Y_FIX_M = -0.010  # 10 mm toward -y = the LM motor (user 2026-11-15 17:4x, after the shoulder was tightened; was +0.003, then 0)


def stand_grasp(cell):
    """Cell's taught grasp pose with STAND_GRASP_Y_FIX_M: [slide, shoulder, elbow, wrist_pitch, tool_roll]."""
    q = taught_target(STAND_GRASP_TAUGHT[cell])
    q[0] -= STAND_GRASP_Y_FIX_M  # +y = -slide
    return q


def taught_target(name):
    """teach.yaml pose -> [slide, shoulder, elbow, wrist_pitch, tool_roll] (MANIPULATOR target)."""
    p = next((p for p in reversed(yaml.safe_load(open(HERE / 'teach.yaml'))) if p['name'] == name), None)  # the latest
    if p is None:
        raise RuntimeError(f'{name} is not in teach.yaml')
    return list(teach.raw_to_target(p['raw_deg']))[:5]


def stand_load_tail(start):
    """Puck held at 75 mm over the box -> clock slot (bigbro load: over the slot, release STAND_RELEASE_MM over the
    sis2 grasp, open, up, carry via, via, nominal)."""
    m = TaskCommand.MANIPULATOR
    over = teach.raw_to_target(PICK_DESCEND_FROM_RAW_DEG + [0.0])[:5]
    mid = teach.raw_to_target(PICK_MID_RAW_DEG + [0.0])[:5]
    over[0] += BIGBRO_SLOT_LM; mid[0] += BIGBRO_SLOT_LM
    f = (STAND_RELEASE_MM - 20.0) / 20.0  # PICK_MID = sis2 grasp + 20, over = + 40
    release = [m_ + (o_ - m_) * f for m_, o_ in zip(mid, over)]
    via = via_pose()[:5]
    return [
        (f'stand load 1: clock to slot {BIGBRO_SLOT} (loading guide; arm over the box)', TaskCommand.CLOCK, BIGBRO_SLOT),
        ('stand load 2: to the carry via pose with the puck (wrist folded, tool_roll 90)', m, carry_via_pose()[:5]),
        (f'stand load 3: over clock slot {BIGBRO_SLOT} (sis2 over pose, 40 mm)', m, over),
        (f'stand load 4: down into the slot to sis2 grasp + {STAND_RELEASE_MM:g} mm', m, release),
        (f'stand load 5: open gripper to {RELEASE_GRIPPER_DEG:g} deg (release in the slot)', TaskCommand.GRIPPER,
         math.radians(RELEASE_GRIPPER_DEG)),
        ('stand load 6: up to 40 mm over the slot', m, over),
        ('stand load 7: to the carry via pose, shoulder out first', m, carry_via_path()),
        ('stand load 8: to the nominal via pose', m, via),
        ('stand load 9: to the nominal pose', m, start),
        ('stand end: hold the nominal pose, shoulder I off', m, start),
    ]


def stand_pick_tail(row, lm, start, cell=None):
    if '--pick' not in sys.argv:
        return []
    if cell not in STAND_GRASP_TAUGHT:
        raise RuntimeError(f'--pick: no taught grasp pose for cell {cell} (STAND_GRASP_TAUGHT: {sorted(STAND_GRASP_TAUGHT)})')
    m = TaskCommand.MANIPULATOR
    roll = teach.raw_to_target([0, 0, 0, 0, STAND_ROLL_RAW_DEG, 0])[4]
    q0 = stand_grasp(cell)  # its roll is replaced by the wait roll + yaw
    setroll = lambda q: list(q[:4]) + [roll]
    app, down, how = descent(setroll(q0), [q0[0]] + list(stand_above_arm(cell)) + [roll])
    q75 = [lm] + teach.raw_to_target([0] + DICE_ROWS[row][75] + [STAND_ROLL_RAW_DEG, 0])[1:5]
    over = teach.raw_to_target(PICK_DESCEND_FROM_RAW_DEG + [0.0])[:5]
    mid = teach.raw_to_target(PICK_MID_RAW_DEG + [0.0])[:5]
    over[0] += BIGBRO_SLOT_LM; mid[0] += BIGBRO_SLOT_LM
    f = (STAND_RELEASE_MM - 20.0) / 20.0  # PICK_MID = sis2 grasp + 20, over = + 40
    release = [m_ + (o_ - m_) * f for m_, o_ in zip(mid, over)]
    via = via_pose()[:5]
    return [
        ('stand pick: open gripper full', TaskCommand.GRIPPER, False),
        (f'stand pick: approach pose over the puck', m, app),
        (f'stand pick: descend onto the puck ({how}; taught {STAND_GRASP_TAUGHT[cell]} + {STAND_GRASP_Y_FIX_M * 1000:g} mm +y, tool_roll from the yaw)', m, down),
        ('stand pick: close gripper (grasp)', TaskCommand.GRIPPER, True),
        ('stand pick: up to 75 mm, one move', m, setroll(q75)),  # puck held: no jaw fix (far row: no IK past it)
    ] + stand_load_tail(start)


# ---- standing-pick calibration (--standcal --cell N, user 2026-11-15) --------------------------------------------
# The upper cam gives a coarse position, the arm goes to the wait pose over it, the gripper cam corrects the rest.
# Per cell (2, 5, 8; centre column) this records what the correction needs, in stand_cal.json:
#   grip_ref: where the gripper cam sees a puck that sits exactly at the taught grasp spot (centre [mm, camera frame],
#             yaw [deg]: the jaws squared it, so this is also the yaw zero check)
#   cam_per_arm: 2x2, gripper-cam centre shift [mm] per arm shift [mm] (columns: LM slide toward the motor, ahead),
#             from shots with the arm moved +-STANDCAL_STEP_MM at the wait pose
#   upper: the upper cam's reading of that same puck from the nominal pose (ahead, lateral [mm]) vs the cell
ROW_AHEAD_MM = {'near': 52.5, 'middle': 112.5, 'far': 172.5}  # LiDAR face -> row centre (dice grid, 2026-11-07)
CELL_ROW = {2: 'far', 5: 'middle', 8: 'near'}
STAND_CAL = Path.home() / 'craft' / 'tools' / 'realsense_check' / 'stand_cal.json'
UPPER_PUCK_CMD = [str(Path.home() / 'craft' / 'tools' / 'realsense_check' / 'build' / 'upper_puck')]
STANDCAL_STEP_MM = 10.0
STANDCAL_SHOTS = {}  # tag -> (centre [x, y, z] mm, yaw deg) / upper reading, filled while it runs


def wait_at(ahead_mm, lm):
    """Wait pose (75 mm over the puck, tool_roll raw STAND_ROLL_RAW_DEG) at any distance ahead between the near and
    the far row: joint-linear between the rows' DICE_ROWS 75 mm poses (outside: the nearest row's)."""
    rows = sorted(ROW_AHEAD_MM, key=ROW_AHEAD_MM.get)
    a = min(max(ahead_mm, ROW_AHEAD_MM[rows[0]]), ROW_AHEAD_MM[rows[-1]])
    for r0, r1 in zip(rows, rows[1:]):
        if a <= ROW_AHEAD_MM[r1]:
            f = (a - ROW_AHEAD_MM[r0]) / (ROW_AHEAD_MM[r1] - ROW_AHEAD_MM[r0])
            q = [x + (y - x) * f for x, y in zip(DICE_ROWS[r0][75], DICE_ROWS[r1][75])]
            return [lm] + teach.raw_to_target([0] + q + [STAND_ROLL_RAW_DEG, 0])[1:5]


def standcal_steps(current_slide=None, start=None):
    """--standcal --cell N (2, 5 or 8): a prism stands roughly square at cell N. nominal -> via -> open -> wait pose ->
    down to the taught grasp -> close (the jaws push it onto the grasp spot and square it) -> open -> up to the wait
    pose -> gripper-cam shot (reference) -> LM +-10 mm, ahead +-10 mm, a shot each -> wait pose -> via -> nominal ->
    upper-cam shot -> solve -> stand_cal.json. The puck stays on the box."""
    start = start if start is not None else nominal_pose()
    cell = int(sys.argv[sys.argv.index('--cell') + 1]) if '--cell' in sys.argv else None
    if cell not in CELL_ROW or cell not in STAND_GRASP_TAUGHT:
        raise RuntimeError(f'--standcal needs --cell 2, 5 or 8 with a taught grasp (got {cell})')
    row = CELL_ROW[cell]
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    lm, _ = dice_lm('centre')
    ahead = ROW_AHEAD_MM[row]
    wait = wait_at(ahead, lm)
    roll = wait[4]
    grasp = stand_grasp(cell)[:4] + [roll]
    st = STANDCAL_STEP_MM
    lo, hi = min(ROW_AHEAD_MM.values()), max(ROW_AHEAD_MM.values())
    da = [d for d in (-st, st) if lo <= ahead + d <= hi]
    da = da if len(da) == 2 else ([-st, -2 * st] if ahead + st > hi else [st, 2 * st])  # one-sided at the end rows
    n = f'standcal c{cell}'
    s = [(f'{n} 0: to the nominal via pose', m, via),
         (f'{n} 0: open gripper full', TaskCommand.GRIPPER, False),
         (f'{n} 1: wait pose over cell {cell} ({row}, {ahead:g} mm ahead)', m, wait),
         (f'{n} 2: down to the taught grasp ({STAND_GRASP_TAUGHT[cell]}), roll raw {STAND_ROLL_RAW_DEG:g}', m, grasp),
         (f'{n} 3: close gripper (centres and squares the puck on the grasp spot)', TaskCommand.GRIPPER, True),
         (f'{n} 3: open gripper full (the puck stays)', TaskCommand.GRIPPER, False),
         (f'{n} 4: up to the wait pose', m, wait),
         (f'{n} 5: gripper-cam reference shot', STANDCAL, {'what': 'grip', 'tag': 'ref', 'dlm': 0.0, 'dahead': 0.0})]
    for d in (st, -st):
        s += [(f'{n} 6: LM {d:+g} mm at the wait pose', m, [lm + d / 1000] + wait[1:]),
              (f'{n} 6: gripper-cam shot, LM {d:+g} mm', STANDCAL, {'what': 'grip', 'tag': f'lm{d:+g}', 'dlm': d, 'dahead': 0.0})]
    for d in da:
        s += [(f'{n} 7: {d:+g} mm ahead at the wait pose', m, wait_at(ahead + d, lm)),
              (f'{n} 7: gripper-cam shot, {d:+g} mm ahead', STANDCAL, {'what': 'grip', 'tag': f'ahead{d:+g}', 'dlm': 0.0, 'dahead': d})]
    s += [(f'{n} 8: back to the wait pose', m, wait),
          (f'{n} 8: to the nominal via pose', m, via),
          (f'{n} 8: to the nominal pose', m, start),
          (f'{n} 9: upper-cam shot of the same puck (arm at the nominal pose)', STANDCAL, {'what': 'upper', 'tag': 'upper'}),
          (f'{n} 10: solve', STANDCAL, {'what': 'solve', 'cell': cell, 'row': row, 'ahead': ahead, 'lm': lm}),
          (f'{n} end: hold the nominal pose, shoulder I off', m, start)]
    return s


def standcal_step(arg):
    """One STANDCAL step: (ok, reason)."""
    import json
    import re
    import subprocess
    num = r'([+-]?\d+(?:\.\d+)?)'
    if arg['what'] == 'grip':
        for k in range(2):
            r = camera_run(STAND_YAW_CMD + ['--save', str(STAND_YAW_SAVE / time.strftime(f"cal_{arg['tag']}_%H%M%S"))],
                               capture_output=True, text=True, timeout=30)
            line = next((l for l in (r.stdout + r.stderr).splitlines() if ': YAW ' in l), (r.stdout + r.stderr).strip()[-200:])
            print('   ' + line.split(': ', 1)[-1])
            mt = re.search(rf'YAW {num} deg CENTRE {num} {num} {num} mm', line)
            if mt:
                yaw, cx, cy, cz = map(float, mt.groups())
                STANDCAL_SHOTS[arg['tag']] = dict(centre=[cx, cy, cz], yaw=yaw, dlm=arg['dlm'], dahead=arg['dahead'])
                return True, f"centre {cx:+.1f} {cy:+.1f} {cz:.1f} mm, yaw {yaw:+.1f}"
        if arg.get('optional'):
            return True, 'gripper cam: no standing prism (2 shots); optional shot, skipped'
        return False, 'gripper cam: no standing prism (2 shots)'
    if arg['what'] == 'upper':
        r = camera_run(UPPER_PUCK_CMD + ['--save', str(STAND_YAW_SAVE / time.strftime('cal_upper_%H%M%S'))],
                           capture_output=True, text=True, timeout=60)
        pucks = []
        for l in r.stdout.splitlines():
            print('   ' + l)
            mt = re.match(rf'PUCK (\S+) {num} ahead {num} lateral {num} top {num}', l)
            if mt:
                pucks.append(dict(cls=mt.group(1), score=float(mt.group(2)), ahead=float(mt.group(3)),
                                  lateral=float(mt.group(4)), top=float(mt.group(5))))
        if not pucks:
            return False, 'upper cam: no puck'
        STANDCAL_SHOTS['upper'] = pucks[0] if len(pucks) == 1 else pucks  # several: all kept, solve picks
        return True, f'{len(pucks)} puck(s)'
    # solve: the wait-pose level, then the approach level (tags ap_*)
    def level(pre):
        ref = STANDCAL_SHOTS.get(pre + 'ref')
        shots = [v for k, v in STANDCAL_SHOTS.items() if k.startswith(pre) and k != pre + 'ref' and k != 'upper'
                 and (pre or not k.startswith(('ap_', 'gr_')))]
        if ref is None or len(shots) < 3:
            return None
        A = np.array([[v['dlm'], v['dahead']] for v in shots])
        B = np.array([np.array(v['centre'][:2]) - ref['centre'][:2] for v in shots])
        X, *_ = np.linalg.lstsq(A, B, rcond=None)  # B = A @ X -> cam_per_arm = X.T
        return ref, X.T, float(np.sqrt(np.mean((A @ X - B) ** 2)))
    w = level('')
    if w is None:
        return False, f'not enough shots ({sorted(STANDCAL_SHOTS)})'
    ref, M, fit = w
    gr = STANDCAL_SHOTS.get('gr_ref')
    up = STANDCAL_SHOTS.get('upper')
    if isinstance(up, list):  # several pucks seen: the one nearest this row
        up = min(up, key=lambda p: abs(p['ahead'] - arg['ahead']) + abs(p['lateral']))
    cal = json.load(open(STAND_CAL)) if STAND_CAL.exists() else {}
    cal[str(arg['cell'])] = dict(
        time=time.strftime('%Y-%m-%d %H:%M:%S'), row=arg['row'], ahead_mm=arg['ahead'], lm=arg['lm'],
        grasp=STAND_GRASP_TAUGHT[arg['cell']], grip_ref=dict(centre_mm=ref['centre'], yaw_deg=ref['yaw']),
        cam_per_arm=M.round(4).tolist(), fit_rms_mm=round(float(fit), 2),
        shots={k: v for k, v in STANDCAL_SHOTS.items() if k != 'upper'},
        grasp_ref=None if gr is None else dict(centre_mm=gr['centre'], yaw_deg=gr['yaw']),
        upper=None if up is None else dict(ahead_mm=up['ahead'], lateral_mm=up['lateral'], cls=up['cls'], score=up['score'],
                                           ahead_err_mm=round(up['ahead'] - arg['ahead'], 1)))
    json.dump(cal, open(STAND_CAL, 'w'), indent=1)
    return True, (f'cam_per_arm {M.round(3).tolist()} (fit rms {fit:.2f} mm), ref yaw {ref["yaw"]:+.1f}; at the grasp pose '
                  + ('no shot' if gr is None else f"centre {gr['centre'][0]:+.1f} {gr['centre'][1]:+.1f}, yaw {gr['yaw']:+.1f}") + '; upper '
                  + ('none' if up is None else f"ahead {up['ahead']:.1f} (cell {arg['ahead']:g}) lateral {up['lateral']:+.1f}")
                  + f' -> {STAND_CAL}')


# ---- --find (user 2026-11-15): upper cam coarse -> wait pose -> gripper cam fine -> pick -> load -------------------
# Upper-cam bias from --standcal 11-15 (cells 8 / 5 / 2: ahead +26.6 / +10.5 / +24.8, lateral -15.9 / -15.6 / -6.4 mm
# vs a puck put on the grasp spot by hand): the mean is taken off; what is left (+-10 mm) the gripper cam corrects.
UPPER_AHEAD_BIAS_MM = 20.1  # cells 8 / 5 / 2: +26.6 / +8.8 (16:35 rerun) / +24.8
UPPER_LATERAL_BIAS_MM = -12.0  # -15.9 / -13.8 / -6.4
FIND_AHEAD_RANGE_MM = (47.5, 177.5)  # cells 8 .. 2 (+-5 mm): the taught rows (user: pucks stay in 2-8)
FIND_MAX_FIX_MM = 25.0               # gripper-cam correction larger than this: stop
FIND = {}                            # this run's state (upper reading, wait pose, ...)
# Every --find run records into runs/find_MMDD_HHMMSS/ (user 2026-11-15: data to tune from): log.json (rewritten after
# every camera step, so a stopped run keeps it), the upper / gripper-cam captures, and two colour photos at the grasp
# pose (jaws open / closed; the gripper cam cannot read depth there). --no-record: no photos (the log stays).
RUN_DIR = (HERE / 'runs' / time.strftime('find_%m%d_%H%M%S')) if '--find' in sys.argv and '--dry-run' not in sys.argv else None


def find_log(**kv):
    """Merge kv into this run's log.json."""
    if RUN_DIR is None:
        return
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    f = RUN_DIR / 'log.json'
    d = json.load(open(f)) if f.exists() else dict(
        start=time.strftime('%Y-%m-%d %H:%M:%S'), argv=sys.argv[1:],
        constants=dict(UPPER_AHEAD_BIAS_MM=UPPER_AHEAD_BIAS_MM, UPPER_LATERAL_BIAS_MM=UPPER_LATERAL_BIAS_MM,
                       STAND_GRASP_Y_FIX_M=STAND_GRASP_Y_FIX_M, STAND_X_FIX_TABLE=STAND_X_FIX_TABLE,
                       STAND_Y_FIX_TABLE=STAND_Y_FIX_TABLE, STAND_YAW_Y_FIX_TABLE=STAND_YAW_Y_FIX_TABLE, STAND_LM_Y_FIX_TABLE=STAND_LM_Y_FIX_TABLE,
                       STAND_YAW_X_FIX_TABLE=STAND_YAW_X_FIX_TABLE,
                       STAND_APPROACH_UP_MM=STAND_APPROACH_UP_MM, STAND_ROLL_PER_YAW=STAND_ROLL_PER_YAW),
        user_eye='(fill in: where the gripper should have gone, e.g. "motor 3, robot 2")')
    d.update(kv)
    json.dump(d, open(f, 'w'), indent=1, default=float)


def find_save_dir():
    if RUN_DIR is None:
        return STAND_YAW_SAVE
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    return RUN_DIR


def fold(a, period=90.0):
    """a [deg] into [-period/2, period/2)."""
    return (a + period / 2) % period - period / 2


def _interp_cells(ahead, value):
    """value(cell) (numbers or lists) linear in the distance ahead between cells 8 / 5 / 2, clamped at the ends."""
    cells = sorted(CELL_ROW, key=lambda c: ROW_AHEAD_MM[CELL_ROW[c]])
    xs = [ROW_AHEAD_MM[CELL_ROW[c]] for c in cells]
    a = min(max(ahead, xs[0]), xs[-1])
    for (c0, x0), (c1, x1) in zip(zip(cells, xs), zip(cells[1:], xs[1:])):
        if a <= x1:
            f = (a - x0) / (x1 - x0)
            v0, v1 = np.asarray(value(c0), float), np.asarray(value(c1), float)
            return v0 + (v1 - v0) * f


# Approach poses (user 2026-11-15): the arm angles of a pose taught 20-30 mm straight over the taught grasp, tool angle
# kept (the wait pose is not over the grasp: going down from it curved and the jaws' inner face caught the puck).
# Only the arm angles count; the LM is the grasp's (from the correction). Cells not taught yet: the wait pose's arm.
STAND_ABOVE_TAUGHT = {2: 'stand_above_c2', 5: 'stand_above_c5', 8: 'stand_above_c8'}  # all taught (c5 17:04, c8 the 17:08 one)


def stand_above_arm(cell):
    """[shoulder, elbow, wrist_pitch] of the cell's approach pose (the wait pose's while it is not taught)."""
    if cell in STAND_ABOVE_TAUGHT:
        return taught_target(STAND_ABOVE_TAUGHT[cell])[1:4]
    line = line_up(stand_grasp(cell))  # not taught: the top of the straight line over its grasp (model), if any
    if line is not None:
        return line[-1][1:4]
    return wait_at(ROW_AHEAD_MM[CELL_ROW[cell]], 0.0)[1:4]


STAND_LINE_MM = 40.0      # straight descent: from this far over the grasp (tool angle kept), in STAND_LINE_PIECES
STAND_LINE_PIECES = 4
STAND_DESCEND_PIECE_S = 0.6  # [s] per piece


def _plane_state(q):
    """(y, z, tool angle) of the TCP in the arm plane (URDF model)."""
    ch = plan.Chain(plan.URDF)
    T = ch.fk(dict(zip(plan.Chain.NAMES, [q[0] - LM_BACKLASH] + list(q[1:5]))))
    return np.array([T[1, 3], T[2, 3], math.atan2(T[2, 1], T[1, 1])])


def line_up(q, dz_mm=None, n=None):
    """Poses from q straight up dz_mm with the tool angle kept (URDF model, small steps from the real grasp pose: the
    model's absolute pose is off, its local motion is not), n pieces, bottom to top; None when the model cannot (near
    its stretched-out singularity, e.g. the far row's grasp) or a joint limit is in the way."""
    dz_mm = STAND_LINE_MM if dz_mm is None else dz_mm
    n = STAND_LINE_PIECES if n is None else n
    ch = plan.Chain(plan.URDF)
    q = list(q[:5]); s0 = _plane_state(q); out = []
    for k in range(1, n + 1):
        tgt = s0 + np.array([0.0, dz_mm * k / n / 1000, 0.0])
        for _ in range(20):
            e = tgt - _plane_state(q)
            if np.linalg.norm(e[:2]) < 2e-5 and abs(e[2]) < 2e-4:
                break
            J = np.zeros((3, 3))
            for j in range(3):
                dq = list(q); dq[j + 1] += 1e-5
                J[:, j] = (_plane_state(dq) - _plane_state(q)) / 1e-5
            if np.linalg.cond(J) > 100:
                return None
            d = np.clip(np.linalg.solve(J, e), -0.05, 0.05)
            q = [q[0]] + [q[i + 1] + d[i] for i in range(3)] + [q[4]]
        e = tgt - _plane_state(q)
        if np.linalg.norm(e[:2]) > 5e-4 or not all(ch.limits(nm)[0] <= q[i + 1] <= ch.limits(nm)[1]
                                                  for i, nm in enumerate(plan.Chain.NAMES[1:4])):
            return None
        out.append([float(v) for v in q])
    return out


STAND_GRASP_UP_MM = 0.0  # grasp over the taught height (0: the taught height; the 3 mm went to the approach, user 11-15)
STAND_APPROACH_UP_MM = 3.0  # approach pose this much higher (user 2026-11-15: margin for the ~2 mm curve on the way down)


def grasp_up(grasp, approach):
    """The grasp STAND_GRASP_UP_MM higher: straight up with the tool angle kept (model), or where the model has no
    line, that fraction of the way toward the approach pose (its height from the model)."""
    if STAND_GRASP_UP_MM <= 0:
        return [float(v) for v in grasp]
    up = line_up(grasp, STAND_GRASP_UP_MM, 1)
    if up is not None:
        return up[0]
    h = (_plane_state(approach)[1] - _plane_state(grasp)[1]) * 1000
    f = min(max(STAND_GRASP_UP_MM / h, 0.0), 0.2) if h > 1 else 0.0
    return [g + (a - g) * f for g, a in zip(grasp, approach)]


def descent(grasp, approach):
    """(approach pose, descent poses, note). The approach is the given one (taught approach poses first, user
    2026-11-15: "why compute when I taught it"; the model's straight line only for a cell not taught), made
    STAND_APPROACH_UP_MM higher (model, tool angle kept; else that fraction further from the grasp), then one move."""
    up = line_up(approach, STAND_APPROACH_UP_MM, 1)
    if up is None:
        h = (_plane_state(approach)[1] - _plane_state(grasp)[1]) * 1000
        f = min(STAND_APPROACH_UP_MM / h, 0.2) if h > 1 else 0.0
        up = [[a + (a - b) * f for a, b in zip(approach, grasp)]]
    return [float(v) for v in up[0]], [grasp_up(grasp, approach)], f'one move from the approach +{STAND_APPROACH_UP_MM:g} mm'


def approach_at(ahead, lm, roll):
    """Approach pose over the grasp at any distance ahead: the approach arms joint-linear between the rows, the
    LM of grasp_at (so the last move down is the arm only)."""
    g = grasp_at(ahead, lm, roll)
    a = _interp_cells(ahead, stand_above_arm)
    return [g[0]] + [float(v) for v in a] + [roll]


def grasp_at(ahead, lm, roll):
    """Grasp pose at any distance ahead: the taught grasps (with the +y fix) joint-linear between the rows; their LM
    relative to the wait LM kept."""
    lm_c = dice_lm('centre')[0]
    g = _interp_cells(ahead, lambda c: [stand_grasp(c)[0] - lm_c] + stand_grasp(c)[1:4])
    return [lm + float(g[0])] + [float(v) for v in g[1:4]] + [roll]


POLICY_TARGET = None
POLICY_CANDIDATES = []


def survey_pick():
    """Fresh box survey before EACH mission pick: standing first, then planar base distance."""
    global POLICY_CANDIDATES
    expected = int(sys.argv[sys.argv.index('--expected-pucks')+1]) if '--expected-pucks' in sys.argv else None
    for attempt in range(2):
        r = camera_run(UPPER_PUCK_CMD + ['--save', str(find_save_dir() / time.strftime(f'survey_%H%M%S_{attempt}'))],
                       capture_output=True, text=True, timeout=60)
        print(r.stdout)
        found = policy_candidates(r.stdout, UPPER_AHEAD_BIAS_MM, UPPER_LATERAL_BIAS_MM)
        try:
            chosen = policy_select(found, expected)
            POLICY_CANDIDATES = found
            break
        except ValueError:
            if attempt: raise
    find_log(selection=dict(rule='standing then distance from base_link', candidates=[c.record() for c in found],
                            picked=chosen.record()))
    print(f'PICK POLICY: {chosen.cls} {chosen.posture}, distance {chosen.distance:.1f} mm '
          f'(ahead {chosen.ahead:.1f}, lateral {chosen.lateral:+.1f}); {len(found)} detected')
    # Check AFTER ranking. Never silently pick a lower-priority puck because the first is out of reach.
    if not FIND_AHEAD_RANGE_MM[0] <= chosen.ahead <= FIND_AHEAD_RANGE_MM[1]:
        raise RuntimeError('highest-priority puck is outside the calibrated forward reach')
    return chosen


def grip_target_args(ahead, posture='standing', cylinder=False):
    cal = json.load(open(STAND_CAL))
    ref = _interp_cells(ahead, lambda c: cal[str(c)]['grip_ref']['centre_mm'][:2])
    if posture == 'lying':
        cell = int(sys.argv[sys.argv.index('--cell')+1])
        floor = cal[str(cell)].get('lie_ref',{}).get('floor_mm')
        if floor:
            ref = np.array(floor[:2])
    args = ['--posture', posture, '--target', str(float(ref[0])), str(float(ref[1])), '40']
    if POLICY_TARGET:
        M = _interp_cells(ahead, lambda c: cal[str(c)]['cam_per_arm'])
        for other in POLICY_CANDIDATES:
            if other == POLICY_TARGET: continue
            d = np.array([-(other.lateral-POLICY_TARGET.lateral), other.ahead-POLICY_TARGET.ahead])
            rival = ref - M @ d
            args += ['--avoid',str(float(rival[0])),str(float(rival[1]))]
    return args + (['--no-yaw'] if cylinder else [])


def find_steps(current_slide=None, start=None):
    """--find: a standing puck anywhere between cells 8 and 2. Fixed skeleton; the poses over the puck are filled in
    while it runs (FIND step 0: upper cam, step 3: gripper cam)."""
    start = start if start is not None else nominal_pose()
    if not STAND_CAL.exists() or not all(str(c) in json.load(open(STAND_CAL)) for c in CELL_ROW):
        raise RuntimeError(f'--find needs {STAND_CAL} with cells {sorted(CELL_ROW)} (--standcal)')
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    lm_c = dice_lm('centre')[0]
    ph = wait_at(112.5, lm_c)  # placeholder until the upper cam has seen the puck
    return [
        ('standfind 0: upper-cam shot (arm at the nominal pose)', STANDFIND, {'what': 'upper'}),
        ('standfind 1: to the nominal via pose', m, via),
        ('standfind 1: open gripper full', TaskCommand.GRIPPER, False),
        ('standfind 2: wait pose over the puck (upper cam)', m, ph),
        ('standfind 3: gripper-cam shot (yaw, centre)', STANDFIND, {'what': 'grip'}),
        ('standfind 4: approach pose over the puck (corrected, jaws turned)', m, ph),
        ('stand pick: descend onto the puck, one move (standfind)', m, ph),
    ] + ([('standfind 5: photo at the grasp pose, jaws open (record)', STANDFIND, {'what': 'photo', 'tag': 'grasp_open'})]
         if '--no-record' not in sys.argv else []) + [
        ('stand pick: close gripper (grasp)', TaskCommand.GRIPPER, True),
    ] + ([('standfind 6: photo, jaws closed (record)', STANDFIND, {'what': 'photo', 'tag': 'grasp_closed'})]
         if '--no-record' not in sys.argv else []) + [
        ('stand pick: up to 75 mm, one move', m, ph),
    ] + stand_load_tail(start)


def _set_pose(steps, i, prefix, q):
    j = next(j for j in range(i + 1, len(steps)) if steps[j][0].startswith(prefix))
    steps[j] = (steps[j][0], steps[j][1], [list(q)])


def find_step(steps, i, arg):
    """STANDFIND: 'upper' fills the wait pose, 'grip' the corrected poses. (ok, reason)."""
    global POLICY_TARGET, POLICY_CANDIDATES
    import re
    import subprocess
    num = r'([+-]?\d+(?:\.\d+)?)'
    lm_c = dice_lm('centre')[0]
    if arg['what'] == 'upper':
        r = type('Survey', (), {'stdout': ''})() if POLICY_TARGET is not None else camera_run(
            UPPER_PUCK_CMD + ['--save', str(find_save_dir() / time.strftime('upper_%H%M%S'))],
            capture_output=True, text=True, timeout=60)
        found = policy_candidates(r.stdout, UPPER_AHEAD_BIAS_MM, UPPER_LATERAL_BIAS_MM)
        chosen = POLICY_TARGET if POLICY_TARGET is not None else policy_select(found)
        if POLICY_TARGET is None:
            POLICY_TARGET, POLICY_CANDIDATES = chosen, found
        if chosen.posture != 'standing':
            return False, 'selected lying prism requires --collect (automatic standing/lying dispatch)'
        score, cls, ahead, lat = chosen.score, chosen.cls, chosen.ahead, chosen.lateral
        cands = [c.record() for c in found]
        if not FIND_AHEAD_RANGE_MM[0] <= ahead <= FIND_AHEAD_RANGE_MM[1]:
            return False, 'highest-priority puck is outside the calibrated forward reach'
        lm = lm_c - lat / 1000  # lateral + = away from the LM motor = -slide
        lo, hi = sorted(teach.raw_to_target([math.degrees(v), 0, 0, 0, 0, 0])[0] for v in teach.motor_limits()[0])
        if not lo <= lm <= hi:
            return False, f'upper cam: puck {lat:+.1f} mm sideways, LM {lm * 1000:.1f} mm is past the rail'
        FIND.update(cls=cls, ahead=ahead, lat=lat, lm=lm)
        _set_pose(steps, i, 'standfind 2:', wait_at(ahead, lm))
        find_log(upper=dict(all=cands, picked=dict(cls=cls, score=score, ahead_mm=ahead, lateral_mm=lat), lm=lm,
                            wait_pose=wait_at(ahead, lm)))
        return True, f'{cls} {score:.2f}: {ahead:.1f} mm ahead, {lat:+.1f} mm sideways (bias taken off) -> LM {lm * 1000:.1f} mm'
    if arg['what'] == 'photo':
        r = camera_run(STAND_YAW_CMD + ['--photo', str(find_save_dir() / arg['tag'])], capture_output=True, text=True, timeout=30)
        ok = 'PHOTO saved' in r.stdout
        find_log(**{f"photo_{arg['tag']}": (arg['tag'] + '_color.png') if ok else 'failed'})
        return True, ('saved' if ok else 'photo failed (record only, going on)') + f' -> {find_save_dir()}'
    if arg['what'] == 'grip3':
        return find_final(steps, i)
    # grip: yaw + centre at the wait pose -> correction
    cal = json.load(open(STAND_CAL))
    yaw = cen = None
    for k in range(2):
        r = camera_run(STAND_YAW_CMD + grip_target_args(FIND['ahead'], cylinder=FIND['cls'].endswith('cylinder')) + ['--save', str(find_save_dir() / time.strftime('grip_wait_%H%M%S'))],
                           capture_output=True, text=True, timeout=30)
        line = next((l for l in (r.stdout + r.stderr).splitlines() if ': YAW ' in l), (r.stdout + r.stderr).strip()[-200:])
        print('   ' + line.split(': ', 1)[-1])
        mt = re.search(rf'YAW {num} deg CENTRE {num} {num} {num} mm', line)
        if mt:
            yaw, cx, cy, _ = map(float, mt.groups()); cen = np.array([cx, cy]); break
    if cen is None:
        return False, 'gripper cam: no standing prism (2 shots)'
    ahead, lm = FIND['ahead'], FIND['lm']
    ref = _interp_cells(ahead, lambda c: cal[str(c)]['grip_ref']['centre_mm'][:2])
    M = _interp_cells(ahead, lambda c: cal[str(c)]['cam_per_arm'])
    yaw0 = float(_interp_cells(ahead, lambda c: cal[str(c)]['grip_ref']['yaw_deg']))
    d_lm, d_ahead = np.linalg.solve(M, ref - cen)  # move the arm so the puck shows where it sat on the grasp spot
    if max(abs(d_lm), abs(d_ahead)) > FIND_MAX_FIX_MM:
        return False, f'gripper cam: correction LM {d_lm:+.1f} ahead {d_ahead:+.1f} mm is over {FIND_MAX_FIX_MM:g} mm'
    yaw_c = float(fold(yaw - yaw0)) if 'prism' in FIND['cls'] else 0.0
    ahead2, lm2 = ahead + d_ahead, lm + d_lm / 1000
    w = wait_at(ahead2, lm2)
    roll = w[4] + math.radians(STAND_ROLL_PER_YAW * yaw_c)
    lo, hi = plan.Chain(plan.URDF).limits('tool_roll_joint')
    if not lo <= roll <= hi:
        return False, f'tool_roll {math.degrees(roll):+.1f} deg is outside its limits'
    ag = ahead2 + stand_x_fix(ahead2) + stand_yaw_x_fix(yaw_c)
    lg = lm2 - (stand_y_fix(ahead2) + stand_yaw_y_fix(yaw_c)) / 1000  # +y = -slide
    app, down, how = descent(grasp_at(ag, lg, roll), approach_at(ag, lg, roll))
    app, app_shift = shift_ahead(app, stand_approach_x_fix(ahead2))
    rail = sorted(teach.raw_to_target([math.degrees(v), 0, 0, 0, 0, 0])[0] for v in teach.motor_limits()[0])
    for nm, q in (('grasp', down[-1]), ('approach', app)):
        if not rail[0] <= q[0] <= rail[1]:
            short = (rail[0] - q[0]) if q[0] < rail[0] else (q[0] - rail[1])
            return False, (f'the puck is out of the LM reach: {nm} LM {q[0] * 1000:.1f} mm, the rail ends at '
                           f'{rail[0] * 1000:.1f} / {rail[1] * 1000:.1f} mm ({short * 1000:.1f} mm short); move the puck inward')
    lo_a, hi_a = min(ROW_AHEAD_MM.values()), max(ROW_AHEAD_MM.values())
    out_note = '' if lo_a - 5 <= ag <= hi_a + 5 else f' !! {ag:.1f} mm is outside the taught rows ({lo_a:g}-{hi_a:g}): poses of the end row'
    _set_pose(steps, i, 'standfind 4:', app)
    j = next(j for j in range(i + 1, len(steps)) if steps[j][0].startswith('stand pick: descend'))
    steps[j] = (f'stand pick: descend onto the puck ({how})', steps[j][1], down)
    _set_pose(steps, i, 'stand pick: up to 75 mm', w[:4] + [roll])
    FIND.update(yaw=yaw_c, d_lm=d_lm, d_ahead=d_ahead, ahead2=ahead2, lm2=lm2, roll=roll)
    find_log(grip=dict(yaw_read=yaw, yaw_zero=yaw0, yaw=yaw_c, centre=cen.tolist(), ref=ref.tolist(),
                       d_lm_mm=d_lm, d_ahead_mm=d_ahead, ahead2=ahead2, lm2=lm2, roll=roll, x_fix_mm=stand_x_fix(ahead2),
                       approach_x_fix_mm=stand_approach_x_fix(ahead2), approach_x_done_mm=app_shift, y_fix_mm=stand_y_fix(ahead2),
                       yaw_y_fix_mm=stand_yaw_y_fix(yaw_c), yaw_x_fix_mm=stand_yaw_x_fix(yaw_c),
                       approach_pose=app, grasp_pose=down[-1]))
    return True, (f'puck off by LM {d_lm:+.1f} mm, ahead {d_ahead:+.1f} mm -> {ahead2:.1f} mm ahead, LM {lm2 * 1000:.1f} mm{out_note}; '
                  f'yaw {yaw:+.1f} - zero {yaw0:+.1f} = {yaw_c:+.1f} deg' + ('' if 'prism' in FIND['cls'] else ' (cylinder: no turn)'))


FIND_FINAL_MAX_MM = 6.0   # final look: a shift up to this is made at the grasp pose with the jaws open; more: stop
FIND_FINAL_MIN_MM = 1.5   # below this no move


def _grip_shot(tag):
    """(yaw, centre [x, y]) from grip_yaw (2 tries) or None."""
    import re
    import subprocess
    num = r'([+-]?\d+(?:\.\d+)?)'
    for k in range(2):
        r = camera_run(STAND_YAW_CMD + ['--save', str(find_save_dir() / time.strftime(f'grip_{tag}_%H%M%S'))],
                           capture_output=True, text=True, timeout=30)
        line = next((l for l in (r.stdout + r.stderr).splitlines() if ': YAW ' in l), (r.stdout + r.stderr).strip()[-200:])
        print('   ' + line.split(': ', 1)[-1])
        mt = re.search(rf'YAW {num} deg CENTRE {num} {num} {num} mm', line)
        if mt:
            yaw, cx, cy, _ = map(float, mt.groups())
            return yaw, np.array([cx, cy])
    return None


def find_final(steps, i):
    """STANDFIND grip3: at the grasp pose with the jaws open, the gripper cam once more vs the grasp-pose reference
    (--standcal 3b: a puck put on the spot by hand, jaws closed and opened). The arm shift from the wait-pose 2x2 (the
    centre is metric, in the camera frame, so a rigid shift reads the same at any height; the tool angle differs
    2-16 deg). A shift between FIND_FINAL_MIN_MM and FIND_FINAL_MAX_MM is made right there, then the jaws close."""
    cal = json.load(open(STAND_CAL))
    if not all(cal.get(str(c), {}).get('grasp_ref') for c in CELL_ROW):
        return True, 'no grasp-pose reference yet (--standcal again): grasp as planned'
    shot = _grip_shot('final')
    if shot is None:
        return True, 'gripper cam at the grasp pose: nothing read (jaws in the way / too near?): grasp as planned'
    _, cen = shot
    ahead, lm, roll = FIND['ahead2'], FIND['lm2'], FIND['roll']
    ref = _interp_cells(ahead, lambda c: cal[str(c)]['grasp_ref']['centre_mm'][:2])
    M = _interp_cells(ahead, lambda c: cal[str(c)]['cam_per_arm'])
    d_lm, d_ahead = np.linalg.solve(M, ref - cen)
    d = max(abs(d_lm), abs(d_ahead))
    if d > FIND_FINAL_MAX_MM:
        return False, f'final look: LM {d_lm:+.1f} ahead {d_ahead:+.1f} mm is over {FIND_FINAL_MAX_MM:g} mm (the jaws would hit it)'
    if d < FIND_FINAL_MIN_MM:
        return True, f'final look: LM {d_lm:+.1f} ahead {d_ahead:+.1f} mm, no move'
    g = grasp_at(ahead + d_ahead, lm + d_lm / 1000, roll)
    steps.insert(i + 1, (f'stand pick: final shift at the grasp pose, LM {d_lm:+.1f} ahead {d_ahead:+.1f} mm',
                         TaskCommand.MANIPULATOR, [grasp_up(g, g)]))
    return True, f'final look: shift LM {d_lm:+.1f} mm, ahead {d_ahead:+.1f} mm before closing'


# ---- lying prism (--lie --cell N, user 2026-11-15) ------------------------------------------------------------------
# A lying prism (30 x 30 x 60, long axis on the box) is picked from above across its 30 mm width, carried to cell 8
# and put down there the same way; then (not written yet) the robot backs off and grasps it again at angles the user
# teaches. Grasp poses taught per cell (lie_grasp_c*: puck long axis along the LM = yaw 0, jaws at roll ~90); the
# approach is the taught lie_above_c* if any, else LIE_LINE_MM straight over the grasp (model; at the lying grasps the
# model has the line at all three cells).
LIE_GRASP_TAUGHT = {2: 'lie_grasp_c2', 5: 'lie_grasp_c5', 8: 'lie_grasp_c8'}
LIE_ABOVE_TAUGHT = {}   # cell -> teach.yaml name once taught
LIE_LINE_MM = 30.0
LIE_PLACE_CELL = 8
LIE_PICK_CELLS = (2, 5)   # cell 8 itself: after a grasp there the roll runs into its limit (user 2026-11-15)
LIE_REGRASP_ABOVE = 'lie_regrasp_above_c8'
LIE_REGRASP_ROLL_RAW_DEG = 180.0  # user 2026-11-15 (taught at 175, the old limit)
LIE_PLACE_YAW_DEG = 90.0
LIE_MAX_LM_FIX_MM = 25.0  # gripper-cam sideways fix of the lying pick larger than this: stop  # put down at cell 8 with the long axis along the robot x (yaw 0 = along the LM)


# --to5 (user 2026-11-16): pick a lying prism from any taught cell, turn the roll by its yaw, put it down at cell 5
# with the long axis along the robot x (LIE_PLACE_YAW_DEG), then shoot it there once to check (no regrasp yet).
# The old flow (put down at 8, drive back, regrasp) stays as it is without --to5 (the mission tests use it).
TO5 = '--to5' in sys.argv
TO5_PLACE_CELL = 5
TO5_PICK_CELLS = (2, 5, 8)
# per-cell pick shift along x [mm] (+ = away from the robot), the picks only (not the put-down, not --lie-ref).
# User's eye, TCP - puck after the run before: 2026-11-16 --to5 cell 2: grasped too far out -> 5 mm toward the robot.
LIE_PICK_X_FIX_MM = {2: -5.0}
LIE_PICK_Y_FIX_MM = 14.0  # lying grasps 17 mm away from the LM motor (user 2026-11-15: 17 was too far -> 14; earlier 10 by eye before the camera fix, then 0)


def lie_grasp(cell, roll=None):
    q = taught_target(LIE_GRASP_TAUGHT[cell])
    if cell in LIE_PICK_CELLS or TO5:  # --to5: every cell (user 2026-11-15: the 14 mm is the same everywhere)
        q[0] -= LIE_PICK_Y_FIX_MM / 1000  # +y = -slide (the put-down at 8 keeps its taught LM)
    return q[:4] + [q[4] if roll is None else roll]


def lie_approach(cell, roll=None):
    g = lie_grasp(cell, roll)
    if cell in LIE_ABOVE_TAUGHT:
        a = taught_target(LIE_ABOVE_TAUGHT[cell])
        return [g[0]] + a[1:4] + [g[4]], f'taught {LIE_ABOVE_TAUGHT[cell]}'
    line = line_up(g, LIE_LINE_MM, 3)
    if line is None:
        raise RuntimeError(f'cell {cell}: no straight line over the lying grasp in the model; teach lie_above_c{cell}')
    return line[-1], f'{LIE_LINE_MM:g} mm straight over the grasp (model)'


def lie_steps(current_slide=None, start=None):
    """--lie --cell N [--pick]: wait pose over cell N -> gripper cam (LYING YAW) -> jaws turned -> approach -> down ->
    close -> up -> (--pick, N != 8) over cell 8 -> down (the same pose, puck lying) -> open -> up to cell 8's approach,
    stop there. Without --pick: stops after the turn."""
    start = start if start is not None else nominal_pose()
    cell = int(sys.argv[sys.argv.index('--cell') + 1]) if '--cell' in sys.argv else None
    pick_cells, place_cell = (TO5_PICK_CELLS, TO5_PLACE_CELL) if TO5 else (LIE_PICK_CELLS, LIE_PLACE_CELL)
    if cell not in pick_cells:
        raise RuntimeError(f'--lie needs --cell {pick_cells} (got {cell})')
    m = TaskCommand.MANIPULATOR
    via = via_pose()[:5]
    lm = lie_grasp(cell)[0]  # the wait pose right over the grasp: the same LM (with LIE_PICK_Y_FIX_MM) for the
    # reference shot and the pick shot (user 2026-11-15: no going back toward the motor to shoot)
    wait = wait_at(ROW_AHEAD_MM[CELL_ROW[cell]], lm)
    if '--lie-ref' in sys.argv:
        # the cell's lying reference (user 2026-11-15): as --standcal, the jaws put the puck on the grasp spot first:
        # wait -> approach -> grasp -> close -> open -> approach -> wait -> shot (saved, then stop)
        app0, how0 = lie_approach(cell)
        return [('lie 0: to the nominal via pose', m, via),
                ('lie 0: open gripper full', TaskCommand.GRIPPER, False),
                (f'lie 1: wait pose over cell {cell}', m, wait),
                (f'lie ref: approach ({how0})', m, app0),
                (f'lie ref: down to the grasp ({LIE_GRASP_TAUGHT[cell]})', m, lie_grasp(cell)),
                ('lie ref: close gripper (puts the puck on the grasp spot, squares it)', TaskCommand.GRIPPER, True),
                ('lie ref: open gripper full (the puck stays)', TaskCommand.GRIPPER, False),
                ('lie ref: up to the approach', m, app0),
                (f'lie ref: back to the wait pose', m, wait),
                ('lie ref: gripper cam reference shot (saved to stand_cal.json lie_ref)', LIEYAW,
                 {'cell': cell, 'roll0': lie_grasp(cell)[4]})]
    if TO5 and '--pick' in sys.argv and 'lie_ref' not in (json.load(open(STAND_CAL)) if STAND_CAL.exists() else {}).get(str(cell), {}):
        # the standing reference dropped to the floor is 17-27 mm off for a lying puck (2026-11-15)
        raise RuntimeError(f'cell {cell} has no lying reference yet: run --lie --cell {cell} --lie-ref first')
    s = [('lie 0: to the nominal via pose', m, via),
         ('lie 0: open gripper full', TaskCommand.GRIPPER, False),
         (f'lie 1: wait pose over cell {cell} (tool_roll raw {STAND_ROLL_RAW_DEG:g})', m, wait),
         ('lie 2: gripper cam reads the lying puck (long axis)', LIEYAW,
          {'cell': cell, 'roll0': lie_grasp(cell)[4], 'place': place_cell})]
    if '--pick' not in sys.argv:
        return s
    app, how = lie_approach(cell)
    grasp = lie_grasp(cell)
    dx = LIE_PICK_X_FIX_MM.get(cell, 0.0)
    if dx:  # the approach and the grasp both, so the descent stays vertical (URDF model, small relative move)
        (app, da), (grasp, dg) = shift_ahead(app, dx), shift_ahead(grasp, dx)
        if abs(da - dx) > 0.05 or abs(dg - dx) > 0.05:
            raise RuntimeError(f'cell {cell}: the {dx:+g} mm x fix reaches only {da:+.1f} / {dg:+.1f} mm in the model')
        how += f', x {dx:+g} mm'
    s += [(f'lie pick: approach over the puck ({how})', m, app),
          (f'lie pick: descend onto the puck, one move ({LIE_GRASP_TAUGHT[cell]})', m, grasp),
          ('lie pick: close gripper (grasp)', TaskCommand.GRIPPER, True),
          (f'lie pick: up to the approach', m, app)]
    app8, how8 = lie_approach(place_cell)
    s += [(f'lie move: over cell {place_cell}, long axis turned to the robot x ({how8})', m, app8),
          (f'lie move: put down at cell {place_cell} ({LIE_GRASP_TAUGHT[place_cell]}), long axis along x', m, lie_grasp(place_cell)),
          ('lie move: open gripper full (release)', TaskCommand.GRIPPER, False),
          (f'lie move: up to cell {place_cell} approach', m, app8)]
    if TO5:
        # check the put-down: the wait pose over cell 5 (its lying reference's LM), one gripper-cam reading vs the
        # target yaw and cell 5's lie_ref; nothing is corrected. Then home.
        wait5 = wait_at(ROW_AHEAD_MM[CELL_ROW[place_cell]], lie_grasp(place_cell)[0])
        return s + [(f'lie check: wait pose over cell {place_cell}', m, wait5),
                    (f'lie check: gripper cam reads the put-down puck (target long axis {LIE_PLACE_YAW_DEG:g})', LIEYAW,
                     {'cell': place_cell, 'roll0': lie_grasp(place_cell)[4], 'check': True}),
                    ('lie end: to the nominal via pose', m, via),
                    ('lie end: to the nominal pose', m, start),
                    ('lie end: hold the nominal pose', m, start)]
    # The regrasp (user 2026-11-15; no wheels yet: the drives are Enter pauses, the user moves the box): back off ->
    # the taught lie_regrasp_above_c8 with tool_roll raw LIE_REGRASP_ROLL_RAW_DEG -> drive forward -> close -> load
    # into clock slot BIGBRO_SLOT the bigbro / standing way (carry via, over the slot, release, home).
    q = taught_target(LIE_REGRASP_ABOVE)
    q[4] = teach.raw_to_target([0, 0, 0, 0, LIE_REGRASP_ROLL_RAW_DEG, 0])[4]
    s += [('lie regrasp 1: [drive back]', PAUSE, 'drive back (move the box: the robot backs off)'),
          # (user 2026-11-15) everything but the shoulder and the roll first; then the shoulder half way; then the rest of
          # the shoulder with the roll (the roll turns only from the shoulder's half way on)
          (f'lie regrasp 2a: elbow, wrist_pitch, LM to the regrasp pose; shoulder and tool_roll stay', m,
           [q[0], app8[1], q[2], q[3], app8[4]]),
          (f'lie regrasp 2b: shoulder half way, tool_roll stays', m,
           [q[0], (app8[1] + q[1]) / 2, q[2], q[3], app8[4]]),
          (f'lie regrasp 2c: shoulder the rest + tool_roll to raw {LIE_REGRASP_ROLL_RAW_DEG:g} ({LIE_REGRASP_ABOVE})', m, q[:5]),
          ('lie regrasp 3: [drive forward]', PAUSE, 'drive forward (move the box: the puck between the jaws)'),
          ('lie regrasp 4: close gripper (grasp)', TaskCommand.GRIPPER, True),
          (f'stand load 1: clock to slot {BIGBRO_SLOT} (loading guide)', TaskCommand.CLOCK, BIGBRO_SLOT)]
    s += [x for x in stand_load_tail(start) if not x[0].startswith('stand load 1:')]
    if POLICY_TARGET is not None:
        # Preserve the taught lying path. Arbitrary forward lying motion is not calibrated yet.
        ref_ahead = ROW_AHEAD_MM[CELL_ROW[cell]] - UPPER_AHEAD_BIAS_MM
        if abs(POLICY_TARGET.ahead-ref_ahead) > 20:
            raise RuntimeError('selected lying prism is outside the taught row neighbourhood (20 mm); teach/calibrate it first')
        lm_delta = dice_lm('centre')[0] - POLICY_TARGET.lateral/1000 - lm
        for j,(name,kind,arg) in enumerate(s):
            if kind == m and name.startswith(('lie 1:', 'lie pick:')):
                q = list(arg);q[0] += lm_delta
                if not -.043 <= q[0] <= .0627:
                    raise RuntimeError('selected lying puck is outside LM reach')
                s[j]=(name,kind,q)
    return s


def lie_place_roll(roll_pick, cell=None):
    """Roll at cell 8 that leaves the puck at LIE_PLACE_YAW_DEG: cell 8's taught roll (yaw 0) + 90 + k*180 (the long
    axis repeats every 180), inside the roll limits, the one nearest the roll it was picked with."""
    lo, hi = plan.Chain(plan.URDF).limits('tool_roll_joint')
    cell = LIE_PLACE_CELL if cell is None else cell
    r0 = lie_grasp(cell)[4] + math.radians(LIE_PLACE_YAW_DEG)
    cands = [r0 + k * math.pi for k in range(-3, 4) if lo <= r0 + k * math.pi <= hi]
    if not cands:
        raise RuntimeError(f'no tool_roll inside the limits puts the puck down along x at cell {cell}')
    return min(cands, key=lambda r: abs(r - roll_pick))


def lie_yaw_step(steps, i, arg):
    """LIEYAW: LYING YAW from the gripper cam (2 shots); roll = the cell's taught grasp roll (puck at yaw 0) + yaw (180
    period: at most 90 either way). Every following 'lie pick:' pose takes that roll; the 'lie move:' poses take
    lie_place_roll, so the puck is put down at cell 8 with its long axis along the robot x every time."""
    import re
    import subprocess
    yaw = floor = None
    for k in range(2):
        r = camera_run(STAND_YAW_CMD + (grip_target_args(ROW_AHEAD_MM[CELL_ROW[arg['cell']]], 'lying') if POLICY_TARGET else []) + ['--save', str(STAND_YAW_SAVE / time.strftime('lieyaw_%H%M%S'))],
                           capture_output=True, text=True, timeout=30)
        line = next((l for l in (r.stdout + r.stderr).splitlines() if 'YAW' in l), (r.stdout + r.stderr).strip()[-200:])
        print('   ' + line.split(': ', 1)[-1])
        mt = re.search(r'LYING YAW ([+-]?\d+(?:\.\d+)?) deg', line)
        if mt:
            yaw = float(mt.group(1))
            mf = re.search(r'FLOOR ([+-]?\d+(?:\.\d+)?) ([+-]?\d+(?:\.\d+)?)', line)
            floor = np.array([float(mf.group(1)), float(mf.group(2))]) if mf else None
            break
        if ': YAW ' in line:
            return False, 'the gripper cam sees a STANDING prism here: use --stand / --find'
    if yaw is None:
        return False, 'no lying prism read (2 shots)'
    prev = next((steps[j][2][-1] for j in range(i - 1, -1, -1) if steps[j][1] == TaskCommand.MANIPULATOR), None)
    lo, hi = plan.Chain(plan.URDF).limits('tool_roll_joint')
    # the taught grasp roll (yaw 0) + the puck's yaw; the long axis repeats every 180, so + k * 180 inside the limits,
    # the one nearest the wait roll (yaw 90: +90 is past the +80.8 limit, -90 is not)
    r1 = arg['roll0'] + math.radians(STAND_ROLL_PER_YAW * yaw)
    cands = [r1 + k * math.pi for k in range(-3, 4) if lo <= r1 + k * math.pi <= hi]
    if not cands:
        return False, f'tool_roll {math.degrees(r1):+.1f} deg (+-180) is outside its limits'
    roll = min(cands, key=lambda r: abs(r - prev[4]))
    place = lie_place_roll(roll, arg.get('place'))
    # sideways (LM) from the gripper cam too (user 2026-11-15): the puck's FLOOR point vs the cell's standing reference
    # dropped to the floor (stand_cal.json grip_ref.floor_mm; the same spot for a standing or lying puck), through the
    # cell's 2x2; only the LM part is used (the ahead part not yet: fixed cells)
    d_lm = d_ahead = 0.0
    cal = json.load(open(STAND_CAL)) if STAND_CAL.exists() else {}
    if '--lie-ref' in sys.argv:  # this shot is the cell's lying reference (the puck put where lie_grasp takes it)
        if floor is None:
            return False, 'no FLOOR in the reading: nothing saved'
        cal.setdefault(str(arg['cell']), {})['lie_ref'] = dict(floor_mm=floor.tolist(), yaw_deg=yaw,
                                                               time=time.strftime('%Y-%m-%d %H:%M:%S'))
        json.dump(cal, open(STAND_CAL, 'w'), indent=1)
        print(f'   lying reference of cell {arg["cell"]} saved: FLOOR {floor[0]:+.1f} {floor[1]:+.1f} mm, yaw {yaw:+.1f}')
        return False, 'reference saved (--lie-ref stops here on purpose)'
    # the cell's own lying reference (--lie-ref) if any, else the standing one dropped to the floor (17-27 mm off,
    # 2026-11-15: the lying and standing grasp spots differ)
    ref = cal.get(str(arg['cell']), {}).get('lie_ref', {}).get('floor_mm') or \
        cal.get(str(arg['cell']), {}).get('grip_ref', {}).get('floor_mm')
    if floor is not None and ref is not None:
        M = np.array(cal[str(arg['cell'])]['cam_per_arm'])
        d_lm, d_ahead = np.linalg.solve(M, np.array(ref[:2]) - floor)
        if abs(d_lm) > LIE_MAX_LM_FIX_MM and not arg.get('check'):
            return False, f'gripper cam: the puck is {d_lm:+.1f} mm off sideways (more than {LIE_MAX_LM_FIX_MM:g}): put it nearer cell {arg["cell"]}'
    if arg.get('check'):  # --to5 put-down check: report only
        dev = (yaw - LIE_PLACE_YAW_DEG + 90) % 180 - 90
        where = (f'grasp spot - puck (the user\'s TCP - puck): x {-d_ahead:+.1f} mm, y {d_lm:+.1f} mm'
                 if floor is not None and ref is not None else 'no FLOOR / lie_ref: position not checked')
        print(f'   PUT-DOWN CHECK cell {arg["cell"]}: long axis {yaw:+.1f} deg (target {LIE_PLACE_YAW_DEG:g}, off {dev:+.1f}); {where}')
        return True, f'put-down check: yaw off {dev:+.1f} deg'
    if POLICY_TARGET and abs(d_ahead) > LIE_MAX_LM_FIX_MM:
        return False, f'selected lying puck needs {d_ahead:+.1f} mm ahead correction, outside local calibration'
    for j in range(i + 1, len(steps)):
        n_, k_, a_ = steps[j]
        if n_.startswith('lie pick:') and k_ == TaskCommand.MANIPULATOR:
            corrected = []
            for pp in a_:
                q = [pp[0]+d_lm/1000] + list(pp[1:4]) + [roll]
                if POLICY_TARGET and abs(d_ahead) >= .05:
                    ok, q = _shift_try(q,d_ahead)
                    if not ok:
                        return False, 'selected lying position cannot be reached by the local taught-pose correction'
                if not -.043 <= q[0] <= .0627:
                    return False, 'corrected lying pose is outside LM reach'
                corrected.append(q+list(pp[5:]))
            steps[j] = (n_, k_, corrected)
        elif n_.startswith(('lie move:', 'lie regrasp 2a', 'lie regrasp 2b')) and k_ == TaskCommand.MANIPULATOR:
            # at cell 8 the long axis along the robot x; the regrasp keeps that roll until the shoulder is half way
            steps[j] = (n_, k_, [list(pp[:4]) + [place] + list(pp[5:]) for pp in a_])
    steps.insert(i + 1, (f'lie: turn the jaws across the puck, tool_roll {math.degrees(prev[4]):+.1f} -> {math.degrees(roll):+.1f} deg '
                         f'(long axis {yaw:+.1f})', TaskCommand.MANIPULATOR, [list(prev[:4]) + [roll]]))
    return True, (f'lying puck, long axis {yaw:+.1f} deg: tool_roll {math.degrees(roll):+.1f} deg; LM {d_lm:+.1f} mm; put down at cell {arg.get('place') or LIE_PLACE_CELL} with '
                  f'tool_roll {math.degrees(place):+.1f} deg (long axis along x)')


def stand_steps(current_slide=None, start=None):
    """--stand: standing-puck pick, step 2 check (user 2026-11-15). A puck assumed on every cell of the dice 3x3 grid;
    for each: via pose -> the pre-descend pose over it (= the dice pose after the release and the ascend: DICE_ROWS
    75 mm, tool vertical, gripper full open) but tool_roll
    STAND_ROLL_RAW_DEG (90 deg turned from the dice put-down). -> back to the via pose. No descent.
    Order as the dice put-down: near row first, then middle, far; left, centre, right."""
    start = start if start is not None else nominal_pose()
    via = via_pose()
    m = TaskCommand.MANIPULATOR
    # Cell numbers (user 2026-11-15, robot frame, seen from behind the robot like DICE_FACES): 1 2 3 = far row,
    # 4 5 6 = middle, 7 8 9 = near; left to right. (Before 11-15 18h: 1-3 were the near row, so the bags
    # gripyaw_1115_0352 / 0357 labelled 8 / 2 were far / near centre = 2 / 8 now.)
    cells = sorted(((r, c) for r in DICE_ROWS for c in DICE_COL_STEPS), key=lambda p: (-_ROW_ORDER[p[0]], _COL_ORDER[p[1]]))
    one = int(sys.argv[sys.argv.index('--cell') + 1]) if '--cell' in sys.argv else None  # --cell N (1-9): go there and stay
    if one is not None:
        cells = [cells[one - 1]]
    total = len(cells)
    s = [('stand 0: to the nominal via pose', m, via[:5]),
         ('stand 0: open gripper full', TaskCommand.GRIPPER, False)]
    for k, (row, col) in enumerate(cells, 1):
        lm, ff = dice_lm(col)
        q = [lm] + teach.raw_to_target([0] + DICE_ROWS[row][75] + [STAND_ROLL_RAW_DEG, 0])[1:5]
        n = f'stand {one or k}/9'
        if one is not None:  # stays over the puck (gripper-cam yaw bags); --goto basic brings it home
            s.append((f'{n} pre-descend over the {row}-{col} puck (75 mm, tool_roll raw {STAND_ROLL_RAW_DEG:g}{ff})', m, q))
            return s + stand_yaw_tail() + stand_pick_tail(row, lm, start, one)
        s += [(f'{n} pre-descend over the {row}-{col} puck (75 mm, tool_roll raw {STAND_ROLL_RAW_DEG:g}{ff})', m, q),
              (f'{n} back to the nominal via pose', m, via[:5])]
    s.append(('stand end: to the nominal pose', m, start))  # work -> via -> nominal (user 2026-11-13)
    s.append(('stand end: hold the nominal pose, shoulder I off', m, start))
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


CAMERA_RUNNER = None


def camera_run(command, **kwargs):
    """Use the calibrated C++ camera program while the project driver lends its USB camera."""
    import subprocess
    node = CAMERA_RUNNER
    role = 'uppercam' if Path(command[0]).name == 'upper_puck' else 'd435'
    stamp = node.get_clock().now().to_msg() if node else None
    def capture():
        result = subprocess.run(command, **kwargs)
        if node and '--photo' not in command:
            try:
                if role == 'uppercam':
                    msg = upper_observations(result.stdout or '', HERE.parent.parent / 'tools/realsense_check/upper_cam_pose.txt',
                                             UPPER_AHEAD_BIAS_MM, UPPER_LATERAL_BIAS_MM)
                else:
                    msg = grip_observations(result.stdout or '')
                msg.header.stamp = stamp
                node.puck_pubs[role].publish(msg)
            except Exception as e:
                print(f'   puck display: {e}')  # visualization cannot change a pick result
        return result
    if node is None or not node.planned:
        return capture()
    pub = node.camera_pause[role]
    if pub.get_subscription_count() == 0:
        raise RuntimeError(f'{role}: project camera driver with pick_pause support is not running')
    node.camera_ack[role] = False
    node.camera_leased = role
    try:
        pub.publish(Bool(data=True))
        deadline = time.monotonic() + 5
        while not node.camera_ack[role]:
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{role}: camera release acknowledgement timed out')
            time.sleep(.02)
        return capture()
    finally:
        node.camera_leased = None
        pub.publish(Bool(data=False))


class Runner(Node):
    def __init__(self):
        super().__init__('puck_pick_execute')
        self.planned = '--planning' in sys.argv
        self.puck_pubs = {role: self.create_publisher(PuckObservationArray, '/perception/pucks/picker_' + name, 10)
                          for role, name in [('uppercam', 'upper'), ('d435', 'grip')]}
        self.planning_state = None
        self.camera_leased = None
        self.camera_pause, self.camera_ack = {}, {}
        for role, prefix in [('uppercam', '/perception/uppercam'), ('d435', '/vision/d435')]:
            self.camera_pause[role] = self.create_publisher(Bool, prefix + '/pick_pause', 10)
            self.create_subscription(Bool, prefix + '/pick_paused',
                                     lambda m, role=role: self.camera_ack.update({role: m.data}), 10)
        self.create_timer(.2, self.renew_camera)
        global CAMERA_RUNNER
        CAMERA_RUNNER = self
        self.lifecycle = self.create_publisher(String, '/planning/command', 10)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.create_subscription(String, '/planning/state', lambda m: setattr(self, 'planning_state', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd = self.create_publisher(TaskCommand, '/planning/pick_step' if self.planned else '/planning/task_command', 10)
        self.hold = None if self.planned else self.create_publisher(Bool, '/planning/hold', 10)
        self.result, self.status, self.grasped, self.joints = None, None, None, None
        self.create_subscription(JointState, '/control/arm_joints', lambda m: setattr(self, 'joints', m), 10)
        self.create_subscription(TaskResult, '/planning/pick_step_result' if self.planned else '/control/task_result', lambda m: setattr(self, 'result', m), 10)
        self.create_subscription(ControlStatus, '/control/status', lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Bool, '/control/gripper_grasped', lambda m: setattr(self, 'grasped', m.data), 10)
        self.hold_value = False
        if self.hold:
            self.create_timer(0.1, lambda: self.hold.publish(Bool(data=self.hold_value)))  # 10 Hz heartbeat
        self.task_id = int(time.time())  # monotonic across runs; control rejects retired (smaller) IDs

    def renew_camera(self):
        if self.camera_leased:
            self.camera_pause[self.camera_leased].publish(Bool(data=True))

    def send(self, kind, arg, seconds=0.0, tolerance=0.0, i_gains=None, contact=0.0):
        if self.planned:
            deadline = time.monotonic() + 5
            while self.planning_state != 'Run.AwaitRequest':
                if self.planning_state in ('Fault', 'Ready') or time.monotonic() >= deadline:
                    raise RuntimeError(f'planning cannot accept a pick step: {self.planning_state}')
                time.sleep(0.02)
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
        elif kind == TaskCommand.NAVIGATE and self.planned:
            c.header.frame_id, c.speed = arg
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

    def drive(self, reverse):
        box = sys.argv[sys.argv.index('--box') + 1]
        result, reason = self.wait(self.send(TaskCommand.NAVIGATE, (box, -0.28 if reverse else 0.0)), timeout=180)
        print(f'   -> {result}: {reason}')
        return result == 'SUCCEEDED'

    def stop(self):
        if self.planned:
            self.lifecycle.publish(String(data='STOP'))
            return
        c = TaskCommand(); c.operation = TaskCommand.STOP; c.task_id = 0
        self.cmd.publish(c); self.hold_value = True


def describe(kind, arg):
    if kind == TaskCommand.MANIPULATOR:  # a path of poses after stage()
        return [round(v, 4) for v in arg[-1]] + ([f'{len(arg)} moves'] if len(arg) > 1 else [])
    if kind == TaskCommand.CLOCK:
        return f'slot {arg}'
    if kind == STANDFIND:
        return 'upper cam (upper_puck C++) -> wait pose' if arg['what'] == 'upper' else 'gripper cam (grip_yaw C++) -> corrected poses'
    if kind == STANDCAL:
        return {'grip': 'gripper-cam shot (grip_yaw C++)', 'upper': 'upper-cam shot (upper_puck C++)',
                'solve': f'solve, write {STAND_CAL.name}'}[arg['what']] + (f" [{arg.get('tag')}]" if arg.get('tag') else '')
    if kind == PAUSE:
        if '--planning' in sys.argv:
            return 'planning: reverse 0.28 m' if arg.startswith('drive back') else 'planning: re-dock at --box'
        return 'wait for Enter (always)'
    if kind == LIEYAW:
        return 'grip_yaw (C++, depth): LYING YAW + centre, then tool_roll + yaw (the jaws across the long axis)'
    if kind == STANDYAW:
        return f'grip_yaw (C++, depth), then tool_roll + {STAND_ROLL_PER_YAW:g} x yaw (none under {STAND_YAW_MIN_DEG:g} deg)'
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
LM_START_TOL = float(sys.argv[sys.argv.index('--lm-tol-mm') + 1]) / 1000 if '--lm-tol-mm' in sys.argv else 0.020  # +-20 mm (user 2026-11-15: 1 cm more each way; was 10)
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


STANDFIND = 'standfind'  # pseudo step kind (--find): upper-cam shot / gripper-cam shot that fill in the poses
STANDCAL = 'standcal'   # pseudo step kind (--standcal): a gripper-cam / upper-cam shot or the solve
PAUSE = 'pause'         # pseudo step kind: waits for Enter even in choi / auto mode (a stand-in for a drive, user 2026-11-15)
LIEYAW = 'lieyaw'       # pseudo step kind (--lie): the gripper cam reads the lying puck's long axis, the jaws turn to it
STANDYAW = 'standyaw'   # pseudo step kind (--stand --yaw): the gripper cam reads the prism's yaw, the jaws turn to it
# Standing prism (2026-11-15): tools/realsense_check/grip_yaw.py (venv python: its OpenCV places the tag corners
# better) at the pick wait pose. yaw + = the prism turned clockwise seen from above; tool_roll + turns the jaws
# clockwise seen from above (URDF FK at the wait poses), so the jaws follow with roll = wait roll + yaw. yaw 0 = the
# faces square to the jaws at STAND_ROLL_RAW_DEG (bags: jaws close along the picture x, yaw 0 tag edges along it).
# STAND_ROLL_PER_YAW: the URDF has the roll axis 16-29 deg off vertical there, so 45 deg of roll may turn the jaws
# 41-49 deg seen from above (which tool axis the jaws close along is not certain in the model): 1.0 until the
# live check says otherwise.
# C++ (2026-11-15: the Python capture + reading took ~3 s at the wait pose and the shoulder could not hold): depth
# 640x360, 10 frames (no warm-up), ~0.1 s compute; same algorithm and grip_yaw.json as grip_yaw.py (bags: within 0.6 deg).
STAND_YAW_CMD = [str(Path.home() / 'craft' / 'tools' / 'realsense_check' / 'build' / 'grip_yaw')]
STAND_YAW_SAVE = Path.home() / 'craft' / 'tools' / 'realsense_check' / 'out'  # each reading's depth frames (gripyaw_HHMMSS_*)
# The gripper is bent (user 2026-11-15): plunged vertically with tool_roll at the nominal pose's roll it takes the puck
# STAND_JAW_OFF_M toward the LM motor (+slide) from the TCP. The offset rides on the tool, so at another roll it
# points elsewhere (raw 90 = a quarter turn: mostly ahead / back). jaw_fix() shifts a pick pose by minus that vector,
# found with FK at the pose's own roll: along the LM by the slide, the rest by the arm (ik_top_down, tool angle kept).
STAND_JAW_OFF_M = 0.004
STAND_ROLL_PER_YAW = 1.0
STAND_YAW_MIN_DEG = 1.0  # below this no turn
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


def stand_yaw_step(steps, i):
    """STANDYAW: read the yaw (C++ grip_yaw, depth, up to 2 shots) and insert the jaws' turn after step i: the last
    MANIPULATOR pose before it with tool_roll + STAND_ROLL_PER_YAW * yaw. (ok, reason)."""
    import re
    import subprocess
    yaw, line = None, ''
    for k in range(2):
        r = camera_run(STAND_YAW_CMD + ['--save', str(STAND_YAW_SAVE / time.strftime('gripyaw_%H%M%S'))],
                           capture_output=True, text=True, timeout=30)
        text = r.stdout + r.stderr
        line = next((l for l in text.splitlines() if ': YAW ' in l), text.strip()[-200:])
        print('   ' + line.split(': ', 1)[-1])
        mt = re.search(r'YAW ([+-]?\d+(?:\.\d+)?) deg', line)
        if mt:
            yaw = float(mt.group(1)); break
    if yaw is None:
        return False, 'no standing prism read in the depth (2 shots)'
    prev = next((steps[j][2][-1] for j in range(i - 1, -1, -1) if steps[j][1] == TaskCommand.MANIPULATOR), None)
    if prev is None:
        return False, 'no arm pose before the yaw step'
    if abs(yaw) < STAND_YAW_MIN_DEG:
        return True, f'yaw {yaw:+.1f} deg: no turn'
    q = list(prev)
    q[4] += math.radians(STAND_ROLL_PER_YAW * yaw)
    lo, hi = plan.Chain(plan.URDF).limits('tool_roll_joint')
    if not lo <= q[4] <= hi:
        return False, f'yaw {yaw:+.1f} deg: tool_roll {math.degrees(q[4]):+.1f} deg is outside its limits'
    for j in range(i + 1, len(steps)):  # the pick moves keep the turned roll (the roll turns about the tool axis)
        n_, k_, a_ = steps[j]
        if n_.startswith('stand pick:') and k_ == TaskCommand.MANIPULATOR:
            steps[j] = (n_, k_, [list(pp[:4]) + [q[4]] + list(pp[5:]) for pp in a_])  # taught pose: only the roll
    steps.insert(i + 1, (f'stand yaw: turn the jaws to the prism, tool_roll {math.degrees(prev[4]):+.1f} -> '
                         f'{math.degrees(q[4]):+.1f} deg (yaw {yaw:+.1f})', TaskCommand.MANIPULATOR, [q]))
    return True, f'yaw {yaw:+.1f} deg: jaws turn {STAND_ROLL_PER_YAW * yaw:+.1f} deg next'


def run_steps(node, steps):
    """Run steps one Enter at a time ('bro' runs the rest without asking; 'choi' / --auto on the command line: all of
    them from the start); False if the user quit or a step did not succeed."""
    auto = AUTO
    grasped_at_close = None
    box_known = True
    fixed, tries, shelf_fix = np.zeros(2), 0, 0.0  # this puck's arm correction so far (sideways, ahead, up) [mm], corrections done
    roll_fix = None  # this puck's tool_roll after a gripper-cam turn correction [rad]
    for i, (name, kind, arg) in enumerate(steps):  # GRIPCHECK may insert steps after itself
        if name.startswith('lie regrasp 1:'):
            box_known = False  # moved by drive or by the user; no pose measurement here
        elif name.startswith('lie regrasp 3:'):
            box_known = True  # restored after successful re-docking below
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
        if kind == STANDFIND:
            ok, reason = find_step(steps, i, arg)
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            continue
        if kind == STANDCAL:
            ok, reason = standcal_step(arg)
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            continue
        if kind == PAUSE:
            if node.planned:
                if not name.startswith(('lie regrasp 1:', 'lie regrasp 3:')):
                    raise RuntimeError(f'unhandled mission pause: {name}')
                if not node.drive(name.startswith('lie regrasp 1:')):
                    return False
                continue
            if input(f'   {arg} -- Enter when done (q = stop): ').strip().lower() == 'q':
                return False
            continue
        if kind == LIEYAW:
            ok, reason = lie_yaw_step(steps, i, arg)
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            continue
        if kind == STANDYAW:
            ok, reason = stand_yaw_step(steps, i)
            print(f'   -> {"SUCCEEDED" if ok else "FAILED"} {reason}')
            if not ok:
                print('step did not succeed')
                return False
            continue
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
                secs = STAND_DESCEND_PIECE_S if name.startswith('stand pick: descend') else DESCEND_PIECE_S
                if k < len(arg) - 1:  # passing poses on the way down: no waiting for 0.5 deg; the grasp keeps it
                    tol = math.radians(DESCEND_PASS_DEG)
            moves = [(target, secs, tol, False)]
            if kind == TaskCommand.MANIPULATOR and fold_motion.candidate(name):
                def arm_now():
                    fb = getattr(node, 'joints', None)
                    if fb is None:
                        return None
                    vals = dict(zip(fb.name, fb.position))
                    age = node.get_clock().now().nanoseconds - (int(fb.header.stamp.sec) * 1_000_000_000
                                                                + int(fb.header.stamp.nanosec))
                    if (0 <= age <= 500_000_000 and len(fb.name) == len(set(fb.name))
                            and len(fb.name) == len(fb.position) and all(n in vals for n in ARM)):
                        return [vals[n] for n in ARM]
                    return None
                # 2026-11-16: the step before (a via pose) ends at the 3 deg via tolerance, so the arm is still settling
                # while the clearance check runs and the start moved > 0.5 deg -> rejected. Plan again from the new
                # feedback until the arm holds still (up to ~3 s); reject only if it never does.
                for attempt in range(10):
                    start = arm_now()
                    try:
                        moves, why = fold_motion.prepare_motion(name, start, target, secs, tol,
                                                                held=getattr(node, 'grasped', None) is not False,
                                                                box_known=box_known)
                    except ValueError as exc:
                        print(f'   shoulder-last rejected: {exc}')
                        return False
                    if len(moves) <= 1:
                        break
                    now2 = arm_now()
                    if now2 is not None and all(math.isfinite(v) and abs(v - start[j]) <= (.0005 if j == 0 else math.radians(.5))
                                                for j, v in enumerate(now2)):
                        break
                    time.sleep(0.3)
                else:
                    print('   shoulder-last rejected: feedback changed or became stale during clearance check (3 s, arm not settling)')
                    return False
                if attempt:
                    print(f'   shoulder-last: arm settled, planned again ({attempt + 1} tries)')
                if why: print(f'   shoulder-last skipped: {why}')
            for move_target, move_secs, move_tol, intermediate in moves:
                status, reason = node.wait(node.send(kind, move_target, move_secs, move_tol, gains,
                                                     0.0 if intermediate else (step_contact(name) if kind == TaskCommand.MANIPULATOR else 0.0)))
                if status != 'SUCCEEDED':
                    break
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
            # Result and grasp feedback are separate topics; normal hold retains the grasp.
            grasp_deadline = time.monotonic() + 0.5
            while not node.grasped and time.monotonic() < grasp_deadline:
                time.sleep(0.01)
            grasped_at_close = node.grasped
            if node.planned and not grasped_at_close:
                print('mission pick: no confirmed grasp; stopping')
                return False
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
    slide = min(max(now["slide_joint"], -0.0430), 0.0627)
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
    global POLICY_TARGET
    collect = "--collect" in sys.argv
    dry = '--dry-run' in sys.argv
    if '--planning' in sys.argv and '--lie' in sys.argv and '--box' not in sys.argv:
        raise ValueError('--planning --lie requires --box NAME')
    lying = '--lying' in sys.argv
    dice = '--dice' in sys.argv
    bigbro = '--bigbro' in sys.argv
    gripcal = '--gripcal' in sys.argv
    gripaxis = '--gripzero' in sys.argv
    stand = '--stand' in sys.argv
    lie = '--lie' in sys.argv
    standcal = '--standcal' in sys.argv
    find = '--find' in sys.argv or collect
    unload = '--unload' in sys.argv or stand or lie or standcal or find or dice or bigbro or gripcal or gripaxis  # these set their own way out and stop at their own end
    make_steps = (lie_steps if lie else find_steps if find else standcal_steps if standcal else stand_steps if stand else gripzero_steps if gripaxis else gripcal_steps if gripcal else bigbro_steps if bigbro else dice_steps if dice else unload_steps if unload
                  else lying_steps_from_plan if lying else steps_from_plan)
    goto = sys.argv[sys.argv.index('--goto') + 1] if '--goto' in sys.argv else None
    if goto is not None:
        if goto not in ('nominal', 'via', 'basic', 'cell'):
            print('--goto nominal|via|basic|cell'); return 1
        make_steps = lambda *a: ([x for x in stand_steps() if not x[0].startswith('stand 0')] if goto == 'cell' else
                                 [('to the nominal pose', TaskCommand.MANIPULATOR, nominal_pose())]
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
        if not -0.045 <= slide <= 0.064:  # LM min 97.6 deg = -43.0 mm since 2026-11-15
            print('LM position is outside the calibrated range (wrong start position?). Run checkarm; aborting'); return 1
        slide = min(max(slide, -0.0430), 0.0627)
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
        if collect:
            POLICY_TARGET = survey_pick()
            if POLICY_TARGET.posture == 'lying':
                cell = min(LIE_PICK_CELLS, key=lambda c: abs(ROW_AHEAD_MM[CELL_ROW[c]] - UPPER_AHEAD_BIAS_MM - POLICY_TARGET.ahead))
                if '--cell' in sys.argv:
                    sys.argv[sys.argv.index('--cell')+1] = str(cell)
                else:
                    sys.argv += ['--cell',str(cell)]
                if '--pick' not in sys.argv: sys.argv.append('--pick')
                lie = True; make_steps = lie_steps
        steps = make_steps(via_pose()[0], start)
        if goto is None:
            if not unload:  # sis2 sets its own way out and stops at the front
                steps = with_via(steps, start)  # the work starts with the LM at the via pose
            elif all(abs(now[n] - v) < (0.001 if n == 'slide_joint' else math.radians(0.5))
                     for n, v in zip(ARM, via_pose())):
                print('already at the via pose: the move out to it is skipped')
                steps = [s for s in steps if not s[0].startswith(('sis2 1:', 'dice 0:', 'bigbro 0:', 'stand 0: to'))]
        elif goto == 'cell':  # --goto cell --cell N: from where it is (already over the grid) straight to cell N's pick wait pose
            steps = [x for x in stand_steps(None, start) if not x[0].startswith('stand 0')]
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
                    via_speed = STAND_VIA_DEG_S if name.startswith(('stand ', 'standfind ')) else VIA_DEG_S
                    out.append(q); times.append(max(GOTO_MIN_S, deg / (GOTO_DEG_S if into_nominal else via_speed)))
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
            if node.planned and node.planning_state == 'Run.AwaitRequest':
                break
            free_since = (free_since or time.time()) if not node.status.held else None
            if free_since and time.time() - free_since >= 0.5:
                break
            time.sleep(0.05)
        else:
            print('control stays held (hold heartbeat / a stop not finished): nothing sent'); return 1
        if lie and not collect and node.puck_pubs['uppercam'].get_subscription_count():
            # Colour/shape context for the lying-puck display, taken at nominal. No change to the taught pick.
            try:
                camera_run(UPPER_PUCK_CMD, capture_output=True, text=True, timeout=30)
            except Exception as e:
                print(f'   upper-cam display context unavailable: {e}')
        if not run_steps(node, steps):
            if goto is None and not unload:
                go_home(node, start, lying)
            else:
                print('sending STOP + hold'); node.stop()
            return 1
        for pub in node.puck_pubs.values():
            pub.publish(PuckObservationArray())
        if goto is not None:  # via / nom: done, control (and the board) keep holding the pose
            print('done: control keeps holding the pose.')
            return 0
        # The script ends here (2026-11-14: a stopped bigbro left waiting for Ctrl+C kept publishing hold = True, and
        # every later basic / via was stopped at once). Control holds the pose by itself once the heartbeat is gone.
        print('done: control keeps holding the last pose.')
        return 0
    except KeyboardInterrupt:
        print('\nCtrl+C: sending STOP + hold'); node.stop(); time.sleep(0.5)
        return 130
    except Exception:
        node.stop(); time.sleep(0.2)
        raise
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code or 0)  # not sys.exit: the spin thread aborted the interpreter on the way out ("terminate called", core dumped)
