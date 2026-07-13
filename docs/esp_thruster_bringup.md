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

## Flash The ESP32

Open this sketch in Arduino IDE:

```text
firmware/esp32_thruster_pwm/esp32_thruster_pwm.ino
```

Install the `ESP32Servo` library if needed. Select the ESP32 board and upload.

Default pins:

```text
18 19 21 22 23 25 26 27
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

This sends neutral, then briefly commands one thruster at a time to `1600 us`.

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
ros2 run tardigrade_px4 keyboard_cmd_vel
```

Keyboard commands publish `/tardigrade/cmd_vel`; the ESP bridge converts those
commands into PWM.

## Tune The Mixer

Edit:

```text
config/esp_thruster_map.json
```

Each thruster has a `mix` block:

```json
"mix": {"surge": 1.0, "sway": 0.0, "heave": 0.0, "yaw": -1.0}
```

Meaning:

```text
surge   responds to cmd_vel.linear.x
sway    responds to cmd_vel.linear.y
heave   responds to cmd_vel.linear.z
yaw     responds to cmd_vel.angular.z
```

Flip a coefficient sign if that thruster pushes the wrong way. Set coefficients
to `0.0` for axes that thruster should not affect.
