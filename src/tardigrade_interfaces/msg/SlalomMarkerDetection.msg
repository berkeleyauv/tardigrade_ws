builtin_interfaces/Time stamp

uint8 marker_index
bool visible
float32 confidence

# Image-space target center. Use normalized image coordinates in [0, 1].
float32 center_x
float32 center_y

# Robot-relative target estimate. Positive yaw means target is to robot-left.
float32 yaw_error_rad
float32 distance_m

# Alignment target for passing on the marker's left side.
float32 left_side_yaw_error_rad
float32 left_side_lateral_offset_m
