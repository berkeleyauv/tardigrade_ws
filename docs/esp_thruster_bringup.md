# ESP32 Thruster Bringup

This path replaces the Pixhawk/PX4 thrust path with a simple ESP32 PWM adapter.
The Jetson still owns ROS, teleop, prequalification, and autonomy. The ESP32
only receives serial PWM commands and drives ESC signal pins.

```text
/tardigrade/cmd_vel
  -> tardigrade_esp esp_thruster_bridge
  -> USB serial
  -> ESP32
  -> ESC PWM pins
```

For closed-loop depth and leveling tests, the command path is:

```text
/tardigrade/cmd_vel/manual
  -> depth_attitude_controller
  -> /tardigrade/cmd_vel
  -> esp_thruster_bridge
  -> ESP32
```

## Flash The ESP32

Open this sketch in Arduino IDE:

```text
firmware/esp32_thruster_pwm/esp32_thruster_pwm.ino
```

Install the `ESP32Servo` library if needed. Select the ESP32 board and upload.

Current slot and pin order:

```text
slot 1  pin 21  thruster 1  front left vertical
slot 2  pin 19  thruster 2  front right vertical
slot 3  pin 27  thruster 3  back left vectored
slot 4  pin 18  thruster 4  front right vectored
slot 5  pin 5   thruster 5  front left vectored
slot 6  pin 14  thruster 6  back left vertical
slot 7  pin 12  thruster 7  back right vectored
slot 8  pin 26  thruster 8  back right vertical
```

Default PWM behavior:

```text
1500 us neutral
1100 us min
1900 us max
50 Hz servo/ESC signal
500 ms serial timeout returns all thrusters to neutral
```

## Build The ROS Package

Inside the Jetson container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select tardigrade_esp
source install/setup.bash
```

## Find The ESP Serial Port

Inside the container:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

Use the stable `/dev/serial/by-id/...` name if available.

## Bench Test One Thruster At A Time

This sends neutral, then briefly commands one slot at a time to `1600 us`.

```bash
ros2 run tardigrade_esp esp_thruster_test --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p test_us:=1600 \
  -p hold_sec:=1.0
```

Use `test_us:=1400` to test the reverse direction.

## Run The Cmd Vel Bridge

Terminal 1:

```bash
ros2 run tardigrade_esp esp_thruster_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p config_file:=/ws/config/esp_thruster_map.json
```

Terminal 2:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Keyboard commands publish `/tardigrade/cmd_vel`; the ESP bridge converts those
commands into PWM.

Monitoring topics:

```text
/tardigrade/thrusters/pwm
/tardigrade/esp/status
```

These are read-only/debug topics for Foxglove. `/tardigrade/thrusters/pwm`
contains the last PWM command for slots 1-8. `/tardigrade/esp/status` is a JSON
string with serial health, stale-command state, command age, PWM limits, and the
last write error if one occurs.

## Tune The Mixer

Edit:

```text
config/esp_thruster_map.json
```

Each thruster has a `mix` block:

```json
"mix": {"surge": 1.0, "sway": -1.0, "heave": 0.0, "yaw": -1.0}
```

Meaning:

```text
surge   responds to cmd_vel.linear.x
sway    responds to cmd_vel.linear.y
heave   responds to cmd_vel.linear.z
roll    responds to cmd_vel.angular.x
pitch   responds to cmd_vel.angular.y
yaw     responds to cmd_vel.angular.z
```

Flip a coefficient sign if that thruster pushes the wrong way. Set coefficients
to `0.0` for axes that thruster should not affect.

The four non-vertical thrusters are vectored diagonally, not pure forward
thrusters. They point outward from the vehicle interior. The starter mixer
assumes this X-style pattern:

```text
front left vectored    surge +, sway -, yaw -
front right vectored   surge +, sway +, yaw +
back left vectored     surge +, sway +, yaw -
back right vectored    surge +, sway -, yaw +
```

These signs assume PWM above neutral produces the positive force direction for
each listed thruster. The wiring/location map alone cannot establish propeller
polarity. During the one-at-a-time test, reverse every nonzero mix coefficient
for any thruster whose positive force is opposite the assumed direction.

If a command moves the robot opposite of what the key says, flip the relevant
sign for the affected thrusters. For example, if `j` should strafe left but
strafes right, flip all four `sway` signs.

The default test limit is intentionally low:

```json
"max_delta_us": 50
```

That means a full command maps to `1500 +/- 50 us`. Increase it only after
slot order and directions are verified.

## ZED + VectorNav Depth And Level Hold

The fused odometry node publishes ZED position and filtered velocity together
with VectorNav orientation and angular velocity. The controller captures the
current ZED `z`, roll, and pitch when the first fresh manual command arrives.
Place the robot level before starting keyboard control.

Build the changed packages inside the Jetson container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  tardigrade_state_estimation tardigrade_esp tardigrade_teleop tardigrade_bringup
source install/setup.bash
```

Start the ZED wrapper in terminal 1:

```bash
source /ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Start fused ZED + VectorNav odometry in terminal 2. Replace the VectorNav port
with the stable path found under `/dev/serial/by-id/`:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/YOUR_VECTORNAV_PORT baud:=115200 \
  use_zed_orientation_if_imu_stale:=false
```

Mount the VectorNav in FRD orientation:

```text
VectorNav +X  robot forward
VectorNav +Y  robot right
VectorNav +Z  robot down
```

ROS command and `base_link` conventions remain FLU. The EKF launch publishes
the needed `base_link -> vectornav` static transform by default.

Before connecting thruster power, move and tilt the robot by hand while
checking that `z` increases upward and roll/pitch change smoothly:

```bash
ros2 topic echo /tardigrade/state/odometry
```

Start the controller and ESP bridge in terminal 3:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup esp_depth_hold.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_PORT \
  config_file:=/ws/config/esp_thruster_map.json
```

Start keyboard input in terminal 4. It must publish to the controller's manual
input, not directly to the ESP bridge:

```bash
source /ws/install/setup.bash
ros2 run tardigrade_teleop keyboard_cmd_vel --ros-args \
  -p cmd_vel_topic:=/tardigrade/cmd_vel/manual \
  -p linear_step:=0.25 \
  -p vertical_step:=0.05
```

`w/s` command forward/back, `r/f` move the depth target up/down, and space stops
horizontal motion while depth and leveling remain active. Ctrl-C or stale ZED,
VectorNav, or keyboard data causes a zero command.

### First Water Test Adjustments

Keep `depth_ki` at `0.0` initially. Change one behavior at a time by stopping
the launch and restarting it with new arguments:

```bash
ros2 launch tardigrade_bringup esp_depth_hold.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_PORT \
  depth_kp:=0.8 depth_kd:=0.25 max_heave_command:=0.35 \
  roll_kp:=0.8 roll_kd:=0.15 pitch_kp:=0.8 pitch_kd:=0.15 \
  max_attitude_command:=0.25
```

Adjustment rules:

```text
robot corrects depth in the wrong direction   flip all four heave signs
roll correction increases the tilt            flip all four roll signs
pitch correction increases the tilt           flip all four pitch signs
depth response is too weak                     raise max_heave_command, then depth_kp
depth oscillates up and down                    lower depth_kp or raise depth_kd
leveling response is too weak                  raise max_attitude_command, then roll/pitch kp
roll or pitch oscillates                       lower that kp or raise that kd
steady depth error remains after P/D tuning    add depth_ki in 0.02 increments
```

The controller limits are normalized commands. Actual PWM authority is also
limited by `protocol.max_delta_us` in `config/esp_thruster_map.json`. At the
initial value of `50`, `max_heave_command:=0.35` can produce only about 18 us
of depth correction. If that is inside the ESC/thruster deadband, increase
`max_delta_us` to `75`, then `100`, and repeat the test.
