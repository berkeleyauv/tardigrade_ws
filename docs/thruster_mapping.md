# Thruster Mapping

Use this guide when the robot's real wiring does not match the old diagram.
That is fine. What matters is that the physical robot, PX4 actuator setup, and
ROS assumptions agree.

The source-of-truth file is:

```text
config/thruster_map.yaml
```

## What The YAML Means

The current control path is:

```text
keyboard_cmd_vel -> /tardigrade/cmd_vel -> mavlink_pixhawk_interface -> PX4 -> thrusters
```

ROS commands body motion. PX4 decides which physical thrusters produce that
motion. The YAML does not control the robot yet; it documents the physical
setup so the PX4 configuration can be checked and so future ROS allocator code
can use the same map.

Use ROS body-frame FLU in the YAML:

```text
x positive = forward / bow
y positive = left / port
z positive = up
```

For each thruster, record:

- `thruster_id`: your team's physical label on the robot.
- `diagram_label`: old diagram number, if the thruster still corresponds to it.
- `pixhawk_output`: exact signal output, such as `MAIN1`, `MAIN2`, or `AUX1`.
- `physical_location`: where it is mounted on the robot.
- `role`: horizontal, vertical, or angled.
- `position_m`: measured from the chosen vehicle origin, ideally center of mass.
- `force_direction_when_positive`: unit-ish direction in ROS FLU when positive.
- `px4_reversed`: whether PX4 reverses that output.
- `observed_positive_motion`: what happened during the motor test.

## Establish The Physical Map

1. Make the vehicle physically safe.

   Remove props if possible, keep fingers clear, restrain the robot, use the
   lowest useful motor-test power, and be ready to disarm.

2. Label the physical thrusters `1` through `8`.

   The labels do not need to match the old diagram. They just need to be
   visible and stable.

3. Trace each ESC signal wire to the Pixhawk.

   Fill `pixhawk_output` in `config/thruster_map.yaml`. Example: if physical
   thruster 3 is plugged into Pixhawk MAIN5, set `pixhawk_output: MAIN5`.

4. Use QGroundControl actuator or motor test one output at a time.

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

6. Fix wrong behavior at the PX4 actuator setup level first.

   If the wrong thruster moves, the output assignment is wrong. If the right
   thruster moves in the wrong direction, reverse that output or correct the ESC
   direction. The ROS teleop code should not be used to hide a bad physical map.

7. Repeat after Pixhawk reboot.

   The bench Pixhawk currently fails `param save`, so PX4 actuator changes may
   not persist. Keep this YAML current so the setup can be restored quickly.

## Teleop Verification

After the YAML and PX4 actuator setup match, use the no-motion dry run first:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity
```

In another container terminal:

```bash
ros2 run tardigrade_px4 keyboard_cmd_vel
```

The Pixhawk interface should show `cmd_vel_received=...`. With no speed clamps,
the command path is active but all motion is clamped to zero.

Only after that, add very small nonzero clamps and test one axis at a time:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity \
  -p max_forward_speed:=0.10 \
  -p max_lateral_speed:=0.10 \
  -p max_vertical_speed:=0.05 \
  -p max_yaw_rate:=0.20
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

If an axis is wrong, stop and fix the physical/PX4 mapping before increasing
speed limits.
