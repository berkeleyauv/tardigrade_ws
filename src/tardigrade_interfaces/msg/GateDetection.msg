std_msgs/Header header

# True when the gate (at least one post) was found in this frame
bool detected

# True when BOTH posts were found (yaw_signal is only valid then)
bool both_posts

# Perspective yaw cue: (right black panel width / left black panel width) - 1
# > 0 -> vehicle is angled right of gate normal -> rotate left
# < 0 -> vehicle is angled left of gate normal  -> rotate right
# Only valid when both_posts is true.
float32 yaw_signal

# Horizontal offset of gate center from image center, normalized to [-1, 1]
# (+ = gate is right of image center -> strafe right)
float32 lateral

# Gate bounding box width as a fraction of image width [0, 1].
# Larger = closer. Used as a crude distance proxy (no stereo depth needed).
float32 gate_width_frac

# Gate bounding box in pixels (full image coordinates), for debugging
int32 box_x
int32 box_y
int32 box_w
int32 box_h
