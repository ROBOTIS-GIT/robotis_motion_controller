# HandJoints.msg

# Standard header for timestamps and frame_id
std_msgs/Header header

# Fixed-size array of 21 Point32 structs.
# Point32 uses float32 for each coordinate, which is generally 
# sufficient for non-critical robot localization and saves space 
# compared to float64 (Point).
geometry_msgs/Point32[21] joints
