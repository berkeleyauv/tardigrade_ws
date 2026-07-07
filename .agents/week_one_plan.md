# Current Bringup Plan

This file replaced the original week-one plan after the bench work proved a
different hardware path. The old plan centered Micro XRCE-DDS. The current
working path centers ZED + VectorNav odometry and USB MAVLink to the Pixhawk.

## Current Goal

Make the robot repeatably:

1. Start the Jetson container with ZED SDK access.
2. Run the ZED wrapper.
3. Run the VectorNav driver.
4. Publish fused ZED + VectorNav robot odometry.
5. Send that odometry to PX4 over USB MAVLink.
6. Enter external-control/Offboard mode.
7. Arm from ROS.
8. Dry-run teleop through `/tardigrade/cmd_vel`.
9. Map and verify thrusters before commanding real thrust.

The main human-facing runbook is:

```text
docs/jetson_zed_px4_startup.md
```

The thruster setup guide is:

```text
docs/thruster_mapping.md
```

## What Is Proven

- Docker image builds on the Jetson.
- Local development startup is separated from Jetson hardware startup:
  - `docker/compose.yaml` is the laptop/local-safe base container.
  - `docker/compose.jetson.yaml` is the Jetson/ZED/Pixhawk hardware override.
- ZED wrapper source is tracked as Git submodules pinned to the `humble-v4.0.8`
  tag commits, not as manual Jetson-only clones.
- ZED wrapper can publish `/zed/zed_node/pose` when the ZED is stable on USB3.
- VectorNav publishes `/vectornav/imu`.
- `zed_vectornav_odometry` publishes `/tardigrade/state/odometry`.
- `mavlink_pixhawk_interface` connects to the Pixhawk over `/dev/ttyACM0`.
- The interface sends MAVLink visual odometry to PX4.
- The interface can send required PX4 params into RAM.
- The Pixhawk has armed from ROS.
- `/tardigrade/status` reports `armed: true` when arming succeeds.

## Important Known Issue

The current Pixhawk cannot save parameters:

```text
param save
ERROR [parameters] parameter export to /fs/mtd_params failed
```

The board also reported:

```text
mtd status
Device size: 0 Blocks (0 bytes)
```

Because of this, PX4 parameter changes do not survive reboot. The bench
workaround is to start `mavlink_pixhawk_interface` with:

```text
-p configure_px4_params:=true
```

This sends required params into RAM each startup. Do not remove this from the
arming workflow until Pixhawk parameter storage is fixed or the controller is
replaced.

## Standard Arming Terminals

Open the container from the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws \
  docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml run --rm tardigrade
```

For a MacBook or other non-Jetson development machine, use only:

```bash
docker compose -f docker/compose.yaml run --rm tardigrade
```

Do not use the Jetson override on macOS; it intentionally mounts Jetson-specific
hardware paths that Docker Desktop cannot provide.

### Terminal 1: ZED

```bash
cd /ws
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Expected:

```text
Advertised on topic: /zed/zed_node/pose
```

If the ZED says `CAMERA NOT DETECTED` or `CAMERA_REBOOTING`, stop and fix
USB3/power before debugging ROS or PX4.

### Terminal 2: ZED + VectorNav Odometry

```bash
cd /ws
source install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
```

Expected:

```text
Publishing: /tardigrade/state/odometry
```

Check:

```bash
ros2 topic hz /tardigrade/state/odometry
```

Expected rate is around the ZED pose rate, often about 15 Hz with the low-load
configuration.

### Terminal 3: Pixhawk MAVLink Interface

```bash
cd /ws
source install/setup.bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

Expected:

```text
Opening MAVLink serial: /dev/ttyACM0 @ 921600
Services: /tardigrade/set_armed, /tardigrade/set_external_control
Publishing: /tardigrade/status
Sending visual odometry from: /tardigrade/state/odometry
Sent required PX4 params in RAM
```

### Terminal 4: Arm

```bash
cd /ws
source install/setup.bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 topic echo /tardigrade/status
```

Expected after success:

```text
armed: true
external_control_enabled: true
arming_state: 1
last_ack_command=400
last_ack_result=0
```

Disarm:

```bash
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

## Teleop Dry Run

Teleop commands use:

```text
/tardigrade/cmd_vel
```

The command convention is ROS body-frame FLU:

```text
linear.x   forward/back
linear.y   left/right
linear.z   up/down
angular.z  yaw left/right
```

Start the Pixhawk interface in velocity mode with zero default clamps:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity
```

Run keyboard teleop:

```bash
ros2 run tardigrade_px4 keyboard_cmd_vel
```

Expected in the Pixhawk interface logs:

```text
cmd_vel_received=...
```

With default clamps, this proves the ROS command path without commanding real
motion.

## Real-Thrust Preconditions

Do not command real thrust until:

- The vehicle is physically safe.
- The Pixhawk is armed/disarmed repeatably.
- `/tardigrade/state/odometry` is stable.
- `/tardigrade/status` shows fresh visual odometry age.
- `config/thruster_map.yaml` is filled out.
- QGroundControl actuator tests match `config/thruster_map.yaml`.
- Each axis is tested one at a time with tiny clamps.

Then start with small clamps:

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

If the wrong thrusters move, fix PX4 actuator configuration or the physical
wiring map before increasing limits.

## Current Work Split

Backend/PX4:

- maintain `mavlink_pixhawk_interface`,
- keep `/tardigrade/status` useful,
- document required runtime PX4 params,
- decide how to handle broken Pixhawk parameter storage.

State estimation:

- maintain `zed_vectornav_odometry`,
- verify frame conventions,
- tune covariances and stale-sensor behavior,
- keep ZED and VectorNav setup repeatable.

Teleop/control:

- keep `keyboard_cmd_vel` conservative,
- move teleop into `tardigrade_teleop` when stable,
- test velocity mode with zero clamps before nonzero clamps.

Hardware:

- fill `config/thruster_map.yaml`,
- verify every Pixhawk output with QGroundControl motor tests,
- document any output reversals,
- make the wiring reproducible for future teammates.

Perception/autonomy:

- remain off the critical arming path until bringup is repeatable,
- publish robot-level detections or commands,
- never publish PX4 messages directly.

## Non-Goals For The Immediate Bringup

- Full custom EKF.
- Replacing PX4 actuator allocation tonight.
- Autonomous gate behavior.
- Direct thruster control from perception.
- Hiding arming inside launch files.
- Depending on unsaved Pixhawk parameters.

## Guidance For Future Codex/Agent Sessions

- Read `docs/jetson_zed_px4_startup.md` before changing bringup commands.
- Read `docs/thruster_mapping.md` before changing teleop or actuator behavior.
- Keep `docker/compose.yaml` local-safe. Put Jetson/ZED/Pixhawk mounts in
  `docker/compose.jetson.yaml`.
- Keep ZED source as submodules unless the team explicitly changes dependency
  strategy.
- `.legacy_inspect` is a stale gitlink without a `.gitmodules` entry; avoid
  treating it as an active package.
- Keep PX4-specific details inside `tardigrade_px4`.
- Keep robot-level APIs centered on `/tardigrade/*`.
- Do not re-center Micro XRCE-DDS unless the team explicitly revives that path.
- Update these `.agents` docs whenever the bench procedure changes.
