#!/bin/bash
# "리": capture again, analyse the gripper occlusion of the box, open the figure in VSCode.
set -e
cd "$(dirname "$0")"
stamp=$(date +%H%M%S)
./build/rs_snap out/ri_$stamp
/usr/bin/python3 occlusion.py out/ri_$stamp 2>&1 | grep -v -i warn
/usr/bin/python3 boxsize.py out/ri_$stamp 2>&1 | grep -v -i warn
echo "out/ri_${stamp}_occlusion.png"
