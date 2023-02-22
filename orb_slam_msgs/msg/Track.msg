# Track
# ORB SLAM3 Tracking results message

# Camera image's header?
std_msgs/Header header

# Tracking result
geometry_msgs/Pose pose

# Tracking status
int32 state
# Map Changed
bool changed
# Map ID
uint64 map_id
