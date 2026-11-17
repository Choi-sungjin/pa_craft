#!/bin/bash
# "uc": one uppercam picture, opened in VSCode (the user asked for it by typing uc).
# Live uppercam_bridge running -> one frame from /perception/uppercam/image_raw/compressed (the bridge owns the camera);
# otherwise rs_snap straight from the upper D435 (serial 261322075477).
cd "$(dirname "$0")"
mkdir -p out
png=out/uc_$(date +%H%M%S).png
source ~/craft/install/setup.bash 2>/dev/null
if pgrep -f uppercam_bridge >/dev/null; then
  /usr/bin/python3 - "$png" <<'PY' || exit 1
import sys, io, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from PIL import Image
rclpy.init(); n = rclpy.create_node('uc_snap'); got = []
n.create_subscription(CompressedImage, '/perception/uppercam/image_raw/compressed', got.append, qos_profile_sensor_data)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if got: break
if not got: print('uc: no frame from /perception/uppercam/image_raw/compressed in 10 s'); sys.exit(1)
Image.open(io.BytesIO(bytes(got[0].data))).save(sys.argv[1]); print('uc:', sys.argv[1], 'from the uppercam bridge topic')
PY
else
  ./build/rs_snap out/uc_raw 640 360 261322075477 2>&1 | grep -v -i warn || exit 1
  /usr/bin/python3 -I -c 'from PIL import Image; Image.open("out/uc_raw_color.ppm").save("'"$png"'")' || exit 1
  echo "uc: $png from rs_snap"
fi
code -r "$PWD/$png"
