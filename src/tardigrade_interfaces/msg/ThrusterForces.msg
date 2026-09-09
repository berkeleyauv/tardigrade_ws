# Named per-thruster force requests in newtons. Names and forces must have the
# same length and every name must occur exactly once in the vehicle description.
std_msgs/Header header
string[] names
float32[] forces
