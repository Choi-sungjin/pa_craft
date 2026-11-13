#!/bin/bash
# bigbro2: up to two pucks in front -> the next free clock slots (carousel.yaml; 12, 11, then 10, 9, ...), in ONE run.
# One upper-cam shot at the nominal pose plans the pucks (cam_pick.py --count 2: nearest first; a puck too far, too
# close to another or outside the LM rail is skipped with its reason, the robot is re-parked for those). After the
# first puck is in the clock the arm goes carry via -> via -> straight to the second; the nominal pose only at the end.
# Each released puck's class (red/green, cylinder/prism) is written to its slot in carousel.yaml (carousel.py shows it;
# "carousel reset" before a new 12-puck run).
# The second puck is not re-shot (the arm hides it at the via pose): the gripper-cam check stops the run if it moved > 10 mm.
# Needs control up and the arm at the nominal pose; close the live camera windows first. Extra args go to execute.py (e.g. choi).
cd ~/craft && source install/setup.bash
if pgrep -f 'puck_live.py' >/dev/null; then echo "a live camera window holds a camera (upper cam for the puck, gripper cam for the check): close it (q) first"; exit 1; fi
/usr/bin/python3 demo/puck_pick/carousel.py next 1 >/dev/null || { echo "the carousel is full (carousel / carousel reset)"; exit 1; }
/usr/bin/python3 demo/puck_pick/cam_pick.py --capture --count 2 || { echo "nothing to pick here: stopped"; exit 1; }
n=$(/usr/bin/python3 -c "import json; print(len(json.load(open('demo/puck_pick/plan_cam_pucks.json'))))")
slots=$(/usr/bin/python3 demo/puck_pick/carousel.py next $n) || { echo "only slots ${slots:-none} are free for $n pucks: carousel"; exit 1; }
plans=plan_cam.yaml; [ "$n" = 2 ] && plans=plan_cam.yaml,plan_cam2.yaml
echo "=== $n puck(s) -> clock slot(s) $slots ==="
python3 demo/puck_pick/execute.py --bigbro --plan $plans --slot $slots "$@"
