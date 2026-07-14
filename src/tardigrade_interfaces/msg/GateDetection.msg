builtin_interfaces/Time stamp

bool visible
float32 confidence

# Image-space target center. Use normalized image coordinates in [0, 1].
float32 center_x
float32 center_y

# Robot-relative target estimate. Positive yaw means target is to robot-left.
float32 yaw_error_rad
float32 pitch_error_rad
float32 distance_m

# Competition-side labels. Leave false when unknown.
bool survey_and_repair_visible
bool search_and_rescue_visible
string recommended_side
