# Telemetry from the Tardigrade ESP32 flight controller.
# Mirrors the State (0x80) frame in the firmware wire protocol
# (tardigrade_firmware: firmware/include/comms/Protocol.h). Republished by the
# esp_bridge node so Foxglove and rosbag can see and record ESP-side state.

builtin_interfaces/Time stamp   # ROS receive time on the Jetson, not the ESP clock

float32 roll                    # rad
float32 pitch                   # rad
float32 yaw                     # rad
float32 depth                   # m, ENU z (VehicleState.altitude_m; negative = deeper)
float32 vertical_velocity       # m/s, + is up

bool armed
bool state_valid                # attitude estimate is trustworthy
bool altitude_valid             # depth/vertical channel specifically
bool link_ok                    # operator command link is fresh
bool pose_ok                    # Jetson pose link is fresh (robosub)
