# Only a PX4-side, version-specific observer may publish this evidence.
# Header uses synchronized ROS time; observer must publish false on lost evidence.
std_msgs/Header header
string source_session_id
bool received
bool fused
bool reset_valid
uint64 reset_counter
string detail
