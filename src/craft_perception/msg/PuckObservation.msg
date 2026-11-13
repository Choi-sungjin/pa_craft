Detection detection
int32 puck_id
bool tag_valid
# false: tag method: tags disagree, or one cuboid tag alone (end or side face is ambiguous);
#        donut method: no shelf plane / too few puck points in depth.
bool axis_valid
# Camera optical frame [m]. axis is a unit vector; lying pucks point toward image right
# (180 deg symmetric), standing pucks point up. shelf_normal: tag method = configured world up.
geometry_msgs/Point center
geometry_msgs/Vector3 axis
geometry_msgs/Vector3 shelf_normal
bool standing
float64 height_m
float64 plane_rms_m
uint32 points
# Tag method: tags of this puck used (0 = donut method) and their worst disagreement [deg].
uint32 tags
float64 tag_fit_error_deg
