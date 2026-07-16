# VectorNav Attitude Control: Run and Test

The active hardware control path is:

```text
VectorNav /vectornav/imu
  -> vectornav_odometry (frame conversion only)
  -> /tardigrade/state/odometry
  -> depth_attitude_controller (roll/pitch/yaw PID)
  -> /tardigrade/cmd_vel
  -> esp_thruster_bridge
```

No ZED orientation or ROS-TCP package is used by this path. The controller
stops all thrusters if the command or VectorNav-derived odometry becomes stale.

The state-estimation conversion matches the physical VectorNav mounting:

```text
VectorNav +X  robot right
VectorNav -Y  robot forward
VectorNav +Z  robot down
```

## 1. Build

From the workspace on the Jetson or inside the Jetson container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  vectornav_msgs vectornav tardigrade_state_estimation \
  tardigrade_esp tardigrade_bringup tardigrade_mission
source install/setup.bash
colcon test --packages-select tardigrade_esp tardigrade_state_estimation
colcon test-result --verbose
```

## 2. Verify the VectorNav without thrusters

Find its stable port and start only the sensor path:

```bash
ls -l /dev/serial/by-id/
ros2 launch tardigrade_bringup vectornav_state.launch.py \
  port:=/dev/serial/by-id/YOUR_VECTORNAV_PORT baud:=115200
```

In a second terminal:

```bash
source /opt/ros/foxy/setup.bash
source /ws/install/setup.bash
ros2 topic hz /vectornav/imu
ros2 topic hz /tardigrade/state/odometry
ros2 topic echo /tardigrade/state/odometry --once
```

With the robot level, verify roll and pitch are near zero. Using the ROS FLU
right-hand convention, rotate the nose left and confirm yaw increases; tilt
the left side up and confirm roll increases; raise the nose and confirm pitch
decreases. `vn_sensor_msgs` intentionally leaves `/vectornav/imu` in the
VectorNav native NED/FRD convention, and `vectornav_odometry` performs the one
conversion to ENU/FLU. Fix mounting or frame conversion before connecting
thrusters if any axis is wrong or discontinuous.

## 3. Test each attitude axis with propellers removed

Keep propellers removed, make the ESP emergency stop reachable, and launch the
controller/bridge:

```bash
ros2 launch tardigrade_bringup esp_depth_hold.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_PORT
```

Publish a continuous neutral manual command so the controller engages:

```bash
ros2 topic pub -r 10 /tardigrade/cmd_vel/manual geometry_msgs/msg/Twist '{}'
```

Observe the PID output:

```bash
ros2 topic echo /tardigrade/cmd_vel
```

By hand, disturb only one axis at a time. The corresponding `angular.x`,
`angular.y`, or `angular.z` output must oppose the disturbance and settle toward
zero. If it reinforces a disturbance, stop immediately and correct that axis's
thruster mix signs. Stop the publisher and verify `/tardigrade/cmd_vel` becomes
all zeros within 0.5 seconds.

To request a slow yaw while testing, publish a heading-rate request; zeroing it
holds the new VectorNav heading:

```bash
ros2 topic pub -r 10 /tardigrade/cmd_vel/manual geometry_msgs/msg/Twist \
  "{angular: {z: 0.10}}"
```

## 4. Low-power in-water tuning

Start with the defaults (`roll/pitch: kp=0.8, ki=0.05, kd=0.15`; `yaw:
kp=0.7, ki=0.03, kd=0.12`) and command limits of 0.25. Tether the robot and test
one axis at a time. If an axis oscillates, reduce its `kp` by 25%; if it is slow
but does not oscillate, increase `kp` by 10%. Increase `kd` slightly for
overshoot. Leave `ki` small; increase it only for a repeatable steady bias.

Example conservative launch:

```bash
ros2 launch tardigrade_bringup esp_depth_hold.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_PORT \
  max_attitude_command:=0.12 max_yaw_command:=0.12 \
  roll_kp:=0.6 pitch_kp:=0.6 yaw_kp:=0.5
```

## 5. Test the complete autonomous control path

First run readiness checks with no movement:

```bash
ros2 launch tardigrade_bringup prequal_autonomy.launch.py \
  vectornav_port:=/dev/serial/by-id/YOUR_VECTORNAV_PORT \
  esp_port:=/dev/serial/by-id/YOUR_ESP_PORT dry_run:=true
```

Only after the sensor signs, PID correction signs, kill switch, and dry run all
pass, run a short tethered movement test:

```bash
ros2 launch tardigrade_bringup prequal_autonomy.launch.py \
  vectornav_port:=/dev/serial/by-id/YOUR_VECTORNAV_PORT \
  esp_port:=/dev/serial/by-id/YOUR_ESP_PORT dry_run:=false \
  startup_delay_sec:=15.0 forward_command:=0.10 \
  descent_command:=0.08 descent_duration_sec:=2.0 \
  outbound_duration_sec:=3.0 return_duration_sec:=3.0 \
  max_yaw_command:=0.12
```

Keep the physical kill switch in hand. The IMU-only mission uses timed motion;
it cannot measure traveled distance or depth, but all three attitude axes and
heading feedback come exclusively from the VectorNav.
