# VectorNav Attitude Control: Run and Test

The active hardware control path is:

```text
VectorNav /vectornav/imu (native NED/FRD)
  -> vectornav_imu_transform (convention + physical mount)
  -> /tardigrade/sensors/imu (ROS ENU/FLU, base_link)
  -> vectornav_odometry
  -> /tardigrade/state/odometry
  -> depth_attitude_controller (roll/pitch/yaw PID)
  -> /tardigrade/cmd_vel
  -> esp_thruster_bridge
```

No ZED orientation or ROS-TCP package is used by this path. The controller
stops all thrusters if the command or VectorNav-derived odometry becomes stale.

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
ros2 topic hz /tardigrade/sensors/imu
ros2 topic hz /tardigrade/state/odometry
ros2 topic echo /tardigrade/state/odometry --once
```

With the robot level, verify roll and pitch are near zero. Using the ROS FLU
right-hand convention, rotate the nose left and confirm yaw increases; tilt
the left side up and confirm roll increases; raise the nose and confirm pitch
decreases. `vn_sensor_msgs` intentionally leaves `/vectornav/imu` in the
VectorNav native NED/FRD convention. `vectornav_imu_transform` performs the one
conversion to ENU/FLU and corrects the installed backward-facing mount. Fix
mounting or frame conversion before connecting thrusters if any axis is wrong
or discontinuous. See `docs/coordinate_frames.md` for the complete sign test.

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

Start with the defaults (`roll/pitch: kp=0.8, ki=0.0, kd=0.15`; `yaw:
kp=0.7, ki=0.0, kd=0.12`) and command limits of 0.20. Tether the robot and test
one axis at a time. If an axis oscillates, reduce its `kp` by 25%; if it is slow
but does not oscillate, increase `kp` by 10%. Increase `kd` slightly for
overshoot. Leave `ki` small; increase it only for a repeatable steady bias.

Example conservative launch:

```bash
ros2 launch tardigrade_esp xbox_assisted_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_PORT

ros2 service call /tardigrade/control/set_pid_gains \
  tardigrade_interfaces/srv/SetPidGains \
  "{axis: roll, kp: 0.6, ki: 0.0, kd: 0.15, output_limit: 0.12}"
```

Use `docs/pool_teleop.md` for the required deadman, recording, and one-axis
enable procedure.

## 5. Test the complete autonomous control path

The autonomous launch is not load-bearing for the pool tuning session. It must
publish the same explicit control-enable heartbeat as assisted teleop before it
can drive the newly gated controller. Keep mission nodes stopped during manual
tuning.

Its existing dry-run invocation remains useful for non-powered readiness work:

```bash
ros2 launch tardigrade_bringup prequal_autonomy.launch.py \
  vectornav_port:=/dev/serial/by-id/YOUR_VECTORNAV_PORT \
  esp_port:=/dev/serial/by-id/YOUR_ESP_PORT dry_run:=true
```

Do not run this launch with `dry_run:=false` until the mission owns a fresh,
explicit controller-enable publisher and its shutdown path has been tested.
The IMU-only mission uses timed motion; it cannot measure traveled distance or
depth.
