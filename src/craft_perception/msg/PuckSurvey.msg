# Pucks identified so far (AprilTag id + YOLO color/shape); one message per change and on reset.
std_msgs/Header header
uint8 RED=0
uint8 GREEN=1
uint8 CUBOID=0
uint8 CYLINDER=1
int32[] ids          # puck ids seen so far (0..11), ascending
uint8[] colors       # RED / GREEN, same order as ids
uint8[] shapes       # CUBOID / CYLINDER
float32[] scores     # best YOLO confidence of each
bool[] shape_confirmed  # shape from the AprilTag rule (>= 2 tags, or one face-on tag on a long box); else YOLO
bool complete        # all 12 seen
