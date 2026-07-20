# Thruster Mapping

Use this guide when the robot's real wiring does not match the old diagram.
That is fine. What matters is that the physical robot, ESP output map, and ROS
assumptions agree.

The source-of-truth file is:

```text
config/esp_thruster_map.json
```

The current ESP wiring is:

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

The thrusters called `vectored` above are the four outward-pointing angled
thrusters. They provide surge, sway, and yaw through an X-style mix; they are
not mounted parallel to the vehicle centerline. The JSON coefficients assume
that PWM above neutral produces the positive force direction used by the
starter mix. Verify that electrical/propeller polarity one slot at a time and
flip every nonzero coefficient for any thruster whose positive force is
reversed.

## What The YAML Means

The current control path is:

```text
keyboard_cmd_vel -> /tardigrade/cmd_vel -> esp_thruster_bridge -> ESP32 -> thrusters
```

ROS commands body motion. `esp_thruster_bridge` maps those commands to PWM
slots using `esp_thruster_map.json`.

Use ROS body-frame FLU in the YAML:

```text
x positive = forward / bow
y positive = left / port
z positive = up
```

For each thruster, record:

- `thruster_id`: your team's physical label on the robot.
- `diagram_label`: old diagram number, if the thruster still corresponds to it.
- `slot`: ESP PWM output slot.
- `pin`: ESP32 GPIO pin for that slot.
- `physical_location`: where it is mounted on the robot.
- `role`: horizontal, vertical, or angled.
- `position_m`: measured from the chosen vehicle origin, ideally center of mass.
- `force_direction_when_positive`: unit-ish direction in ROS FLU when positive.
- `observed_positive_motion`: what happened during the motor test.

## Establish The Physical Map

1. Make the vehicle physically safe.

   Remove props if possible, keep fingers clear, restrain the robot, use the
   lowest useful motor-test power, and be ready to disarm.

2. Label the physical thrusters `1` through `8`.

   The labels do not need to match the old diagram. They just need to be
   visible and stable.

3. Trace each ESC signal wire to the ESP32 output.

   Fill the ESP slot/pin in `esp_thruster_map.json`.

4. Use `esp_thruster_test` one output at a time.

   For each output, record which physical thruster moves and what direction it
   pushes for a positive command. Do not test multiple thrusters at once while
   mapping.

5. Fill the force direction.

   Use the ROS FLU convention. Examples:

```yaml
force_direction_when_positive:
  x: 1.0
  y: 0.0
  z: 0.0
```

   means positive command pushes the robot forward.

```yaml
force_direction_when_positive:
  x: 0.0
  y: 0.0
  z: 1.0
```

   means positive command pushes the robot upward.

6. Fix wrong behavior at the ESP map or physical wiring level first.

   If the wrong thruster moves, the output assignment is wrong. If the right
   thruster moves in the wrong direction, reverse that output or correct the ESC
   direction. The ROS teleop code should not be used to hide a bad physical map.

7. Repeat after ESP firmware or wiring changes.

   Keep the JSON map current so setup can be restored quickly.

## Teleop Verification

After the ESP slot map matches the physical robot, run the bridge with thruster
power disconnected first:

```bash
ros2 run tardigrade_esp esp_thruster_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p config_file:=/ws/config/esp_thruster_map.json
```

In another container terminal:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

The ESP bridge should log incoming commands and serial writes. With thruster
power disconnected, this verifies the ROS command path without motion.

Only after that, connect power in a physically safe setup and test one output or
one axis at a time:

```bash
ros2 run tardigrade_esp esp_thruster_test --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p test_us:=1600 \
  -p hold_sec:=1.0
```

Expected first-axis checks:

```text
w  -> forward
s  -> backward
j  -> left
l  -> right
r  -> up
f  -> down
a  -> yaw left
d  -> yaw right
```

If an axis is wrong, stop and fix the physical/ESP mapping before increasing
speed limits.
