#!/bin/bash
# bigbro2: two pucks in front -> clock slots 12, then 11, in ONE run. One upper-cam shot at the nominal pose plans both
# pucks (cam_pick.py --count 2: nearest first, measured top heights, pucks within 80 mm stop it). After the first puck
# is in the clock the arm goes carry via -> via -> straight to the second puck; the nominal pose only at the end.
# The second puck is not re-shot (the arm hides it at the via pose): the gripper-cam check stops the run if it moved > 10 mm.
# Needs control up and the arm at the nominal pose; close the live camera windows first. Extra args go to execute.py (e.g. choi).
cd ~/craft && source install/setup.bash
if pgrep -f 'puck_live.py' >/dev/null; then echo "a live camera window holds a camera (upper cam for the puck, gripper cam for the check): close it (q) first"; exit 1; fi
/usr/bin/python3 demo/puck_pick/cam_pick.py --capture --count 2 || { echo "no plan for both pucks: stopped"; exit 1; }
python3 demo/puck_pick/execute.py --bigbro --plan plan_cam.yaml,plan_cam2.yaml --slot 12,11 "$@"
