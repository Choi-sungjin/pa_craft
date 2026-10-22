Detection detection
int32 puck_id
bool tag_valid
# false: puck box found but no shelf plane / too few puck points in depth.
bool axis_valid
# Camera optical frame [m]. axis is a unit vector; lying pucks point toward image right
# (180 deg symmetric), standing pucks use the shelf normal.
geometry_msgs/Point center
geometry_msgs/Vector3 axis
geometry_msgs/Vector3 shelf_normal
bool standing
float64 height_m
float64 plane_rms_m
uint32 points
