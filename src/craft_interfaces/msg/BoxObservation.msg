std_msgs/Header header
uint64 request_id
string source
bool valid
string reason
geometry_msgs/Pose box_pose
float64 gap_m
float64 lateral_m
float64 yaw_rad
bool extrapolated
float64 gap_uncertainty_m

# True only when both finite face endpoints were observed in the LiDAR scan.
# A valid plane/gap alone does not establish the tangential box centre.
bool face_centre_observed
float64 face_centre_uncertainty_m
