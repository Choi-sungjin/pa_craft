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
# Nominal pose (fixed 2026-11-05): carriage lined up with clock 12, raw motor degrees LM, shoulder, elbow,
# wrist_pitch, tool_roll, gripper. Every run starts here and ends here. Reassembled 2026-11-06 (new elbow
# bracket); LM 858.3 = clock 12 after LM and clock were re-aligned (2026-11-06, was 829.7, 941.1); was
# [1030.9, 69.3, 181.0, 195.8, 90.3, 146.7].
# 2026-11-07: timing belt re-fitted and the LM scale re-measured (control.yaml): clock 12 = LM 548.3 (measured); the
# box places are their slide positions [m] in raw degrees again (box centre = LM mid travel 594.1).
NOMINAL_RAW_DEG = [548.3, 84.9, 100.1, 16.8, 90.0, 94.0]  # tool_roll 90 (was 91.1), gripper full open (was 141.9)
# Via pose between the nominal pose and any work, both ways. Shoulder, elbow, wrist_pitch given by hand
# (2026-11-06); LM the old via - nominal difference on the new nominal; tool_roll and gripper as the nominal pose
# (they must not move); LM = nominal (858.3 since 2026-11-06, was 829.7, 940.8). Was [1030.6, 83.4, 101.2, 114.2, 91.8, 149.6].
VIA_RAW_DEG = [548.3, 92.5, 182.9, 104.6, 90.0, 141.9]  # tool_roll 90 (was 91.1)
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
DICE_ROWS = {
    'far': {0: [128.65, 212.37, -95.9], 40: [119.75, 210.33, -98.83], 75: [110.2, 212.18, -97.54]},
    'middle': {0: [103.7, 263.61, -69.61], 40: [106.61, 234.35, -95.95], 75: [97.07, 227.14, -101.71]},
    'near': {0: [88.22, 282.87, -65.83], 40: [89.83, 256.87, -90.22], 75: [84.13, 239.95, -103.84]},
}
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
              'put down', 'rise above the puck', 'sis2 ', 'dice ',
              'via pose:', 'via pose (off')  # out_to_via; the way back to the via pose stays staged
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


def wrist_first(prev, target, what):
    """Step: prev with only the wrist_pitch moved to target's (the elbow follows in the next step)."""
    q = list(prev)
    q[3] = target[3]
    return (f'wrist_pitch to {what} first, the rest stays', TaskCommand.MANIPULATOR, q)


def out_to_via(prev):
    """Steps prev -> via pose, the reverse of the way back: the shoulder alone, then the elbow with LM, tool_roll and
    gripper, then the wrist_pitch alone."""
    via = via_pose()
    shoulder = list(prev[:1]) + [via[1]] + list(prev[2:])
    elbow = via[:3] + [prev[3]] + via[4:]
    m = TaskCommand.MANIPULATOR
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


def plan_pose(name='plan.yaml'):
    """pose(i): frame i of a plan.py output as [slide + LM_BACKLASH, shoulder, elbow, wrist_pitch, tool_roll]."""
    f = yaml.safe_load(open(HERE / name))['frames']
    return lambda i: [f[i]['joints'][ARM[0]] + LM_BACKLASH] + [f[i]['joints'][n] for n in ARM[1:]]


def frames_path(pose, i0, i1):
    """pose(i) for plan frames i0 -> i1 (either direction) every STAGE_SEG frames, both ends included."""
    step = STAGE_SEG if i1 > i0 else -STAGE_SEG
    return [pose(i) for i in list(range(i0, i1, step))[1:] + [i1]]


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
         ('descend', TaskCommand.MANIPULATOR, [grasp]),
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
        half = [(a + b) / 2 for a, b in zip(roll[2:4], start[2:4])]
        wrist = roll[:3] + half[1:] + roll[4:]  # wrist_pitch before the elbow
        s.append(('back 2a: wrist_pitch half way to the start pose (the rest stays)', TaskCommand.MANIPULATOR, [wrist]))
        s.append(('back 2b: elbow half way to the start pose (LM, shoulder, tool_roll stay)',
                  TaskCommand.MANIPULATOR, [roll[:2] + half + roll[4:]]))
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
            (f'{n} to the nominal via pose with the puck', m, via[:5]),
            (f'{n} 75 mm over the {row}-{col} pip (tool_roll to raw {DICE_PLACE_ROLL_RAW_DEG:g}{ff})', m, pose(75)),
            (f'{n} put down on the {row}-{col} pip, one move', m, pose(0)),
            (f'{n} open gripper full (release)', TaskCommand.GRIPPER, False),
            # One move (2026-11-07, the 40 mm stop dropped): TCP up to 7.8 mm off the straight line (near row), along
            # the column; the open jaws stay >= 20 mm from the neighbours 60 mm away.
            (f'{n} up to 75 mm, one move', m, pose(75)),
        ]
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
        ('sis2 8: to the nominal via pose with the puck (gripper stays closed)', m, via[:5]),
    ] + ([] if PLACE_ABOVE_RAW_DEG is None else [
        ('sis2 9: over the place pose', m, above),
        ('sis2 10: descend onto the box (puck touches)', m, place),
        ('sis2 11: open gripper full (release the puck)', TaskCommand.GRIPPER, False),
        (f'sis2 12: tool_roll to raw {PLACE_RELEASE_ROLL_RAW_DEG:g} (90 deg turn, jaws open), the rest stays', m,
         place[:4] + [roll_out]),
        ('sis2 13: ascend over the place pose', m, above[:4] + [roll_out]),
        ('sis2 14: to the nominal pose, one move (gripper full open)', m, start),
    ])


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
    if not isinstance(arg, bool):
        return f'open to {math.degrees(arg):.1f} deg'
    return 'close' if arg else 'open'


# 'choi' or --auto anywhere on the command line (sis3 choi): no Enter between the steps.
AUTO = 'choi' in sys.argv[1:] or '--auto' in sys.argv[1:]


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
STEP_CONTACT = {'put down on the': 0.7}


def step_contact(name):
    """Stop-on-contact ratio of a MANIPULATOR step (STEP_CONTACT), 0 = off; sent as TaskCommand.speed."""
    return next((r for key, r in STEP_CONTACT.items() if key in name), 0.0)


def step_seconds(name):
    """Travel time [s] of a MANIPULATOR step (the first STEP_SECONDS key in its label), 0 = as fast as the caps
    allow."""
    return next((s for key, s in STEP_SECONDS.items() if key in name), 0.0)


def run_steps(node, steps):
    """Run steps one Enter at a time ('bro' runs the rest without asking; 'choi' / --auto on the command line: all of
    them from the start); False if the user quit or a step did not succeed."""
    auto = AUTO
    for name, kind, arg in steps:
        if auto:
            print(f'>> {name}')
        else:
            answer = input(f'>> {name}? ').strip().lower()
            if answer == 'q':
                return False
            auto = answer == 'bro'
        for target in (arg if kind == TaskCommand.MANIPULATOR else [arg]):
            gains = None
            if kind == TaskCommand.MANIPULATOR:
                gains = step_i_gains(name)
                if gains != I_GAINS_BASE:
                    print(f'   I gains {"/".join(map(str, gains))} (STEP_I_GAINS)')
            status, reason = node.wait(node.send(kind, target, step_seconds(name), step_tolerance(name), gains,
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
    unload = '--unload' in sys.argv or dice  # both set their own way out and stop at their own end
    make_steps = dice_steps if dice else unload_steps if unload else lying_steps_from_plan if lying else steps_from_plan
    goto = sys.argv[sys.argv.index('--goto') + 1] if '--goto' in sys.argv else None
    if goto is not None:
        if goto not in ('nominal', 'via'):
            print('--goto nominal|via'); return 1
        make_steps = lambda *a: ([('to the nominal pose', TaskCommand.MANIPULATOR, nominal_pose())]
                                 if goto == 'nominal' else out_to_via(nominal_pose()))
    steps = make_steps() if make_steps is steps_from_plan else stage(make_steps(), None)
    for name, kind, arg in steps:
        print(f'  {name:45s}', describe(kind, arg), f'{step_seconds(name):g} s' if step_seconds(name) else '')
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
        off = [f'{n} {(now[n] - t) * 1000:+.1f} mm' if n == 'slide_joint' else f'{n} {math.degrees(now[n] - t):+.1f} deg'
               for n, t in zip(ARM, start) if abs(now[n] - t) > (0.010 if n == 'slide_joint' else math.radians(90.0))]
        if off:
            print(f'!! too far from the nominal pose ({", ".join(off)}): move it closer by hand; aborting'); return 1
        steps = make_steps(via_pose()[0], start)
        if goto is None:
            if not unload:  # sis2 sets its own way out and stops at the front
                steps = with_via(steps, start)  # the work starts with the LM at the via pose
            elif all(abs(now[n] - v) < (0.001 if n == 'slide_joint' else math.radians(0.5))
                     for n, v in zip(ARM, via_pose())):
                print('already at the via pose: the move out to it is skipped')
                steps = [s for s in steps if not s[0].startswith(('sis2 1:', 'dice 0:'))]
        elif goto == 'nominal':  # wrist_pitch before the elbow
            steps = [wrist_first([now[n] for n in ARM], start, 'the nominal pose')] + steps
        else:  # from where it was put down: shoulder, elbow, wrist_pitch
            steps = out_to_via([now[n] for n in ARM])
        if make_steps is not steps_from_plan:  # sis1 sets its own move order
            steps = stage(steps, [slide] + [now[n] for n in ARM[1:]])
        for name, kind, arg in steps:
            print(f'  {name:45s}', describe(kind, arg))
        if AUTO:
            print('\nAUTO (choi): every step runs without asking. Keep a hand on the 12 V power; Ctrl+C = STOP + hold.')
            for k in (3, 2, 1):
                print(f'  starting in {k} s ...'); time.sleep(1)
        else:
            print('\nKeep a hand on the 12 V power. Enter = run step, q + Enter = quit.\n')
        if not run_steps(node, steps):
            if goto is None and not unload:
                go_home(node, start, lying)
            else:
                print('sending STOP + hold'); node.stop()
        print('done. control keeps holding the last pose; Ctrl+C to exit this script.')
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('\nCtrl+C: sending STOP + hold'); node.stop(); time.sleep(0.5)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
