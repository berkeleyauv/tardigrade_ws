# Scripts

Helper scripts and ROS entry points are listed here so new contributors can
find the important commands without searching through package files.

## Workspace Helpers

```bash
./docker-build.sh
```

Starts the local development container. For interactive Compose runs, this uses
`--service-ports` so host tools can reach ports such as rosbridge `9090`.

```bash
./docker-build.sh --build
```

Builds the Docker image and starts the local development container.

```bash
./docker-build.sh --jetson
```

Starts the Jetson hardware container using the base Compose file plus the
Jetson override. The Jetson override uses host networking, so `docker ps` will
not show per-port mappings.

```bash
./build.sh
```

Builds the workspace for local development. By default this skips ZED SDK
packages that only build on the Jetson or another machine with the Stereolabs
SDK installed.

```bash
./build.sh --hardware
```

Builds all packages, including the ZED SDK packages.

```bash
./build.sh --clean
```

Removes `build`, `install`, and `log`, then rebuilds the local development
workspace.

Build and source the workspace before running these:

```bash
./build.sh
source install/setup.bash
```

## Bringup Launch Files

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

Starts the local mock stack for development without hardware.

```bash
ros2 launch tardigrade_bringup zed_state.launch.py
```

Starts the ZED odometry path.

```bash
ros2 launch tardigrade_bringup vectornav_state.launch.py
```

Starts the VectorNav state path.

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
```

Starts the combined ZED + VectorNav odometry path. This assumes the ZED wrapper
is already publishing `/zed/zed_node/pose`.

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Starts the experimental `robot_localization` EKF path. It assumes the ZED
wrapper is already publishing `/zed/zed_node/odom` and VectorNav is already
publishing `/vectornav/imu`. The default output is
`/tardigrade/state/odometry/filtered` so it can be compared against the current
odometry path before being promoted.

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Starts rosbridge on port `9090` for Foxglove's Rosbridge connection. This is
the current Foxglove MVP path for ROS 2 Foxy.

```bash
ros2 launch tardigrade_bringup foxglove_bridge.launch.py
```

Starts `foxglove_bridge` on port `8765` if the bridge has been built from
source or otherwise installed.

## ESP / Control

```bash
ros2 run tardigrade_esp esp_thruster_bridge
```

Current ESP32 serial bridge. This is the main actuator path for converting
`/tardigrade/cmd_vel` into PWM commands.

Monitoring output:

```text
/tardigrade/thrusters/pwm
/tardigrade/esp/status
```

```bash
ros2 run tardigrade_esp esp_thruster_test
```

Bench utility for testing one ESP thruster output at a time.

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Publishes keyboard velocity commands for bench testing.

## Legacy Pixhawk / PX4

`src/legacy/tardigrade_px4` is preserved for reference and ignored by colcon.
`src/legacy/px4_msgs` is preserved there too. Remove
`src/legacy/COLCON_IGNORE` only if you intentionally want to work on the old
Pixhawk path.

The commands below are legacy references and are not available in the normal
ESP-first build.

```bash
ros2 run tardigrade_px4 motor_toggle_test
```

Small motor toggling utility. Use only when the vehicle is physically safe.

```bash
ros2 run tardigrade_px4 mock_px4_status
```

Publishes mock Pixhawk status for local development.

```bash
ros2 run tardigrade_px4 pixhawk_interface
```

Older PX4 ROS 2 `/fmu/*` interface for mock/uXRCE-style work.

```bash
ros2 run tardigrade_px4 odometry_to_px4
ros2 run tardigrade_px4 mavlink_odometry_to_px4
```

Odometry bridge experiments for sending robot odometry to PX4.

## State Estimation

```bash
ros2 run tardigrade_state_estimation zed_odometry
```

Converts ZED pose into the robot odometry topic.

```bash
ros2 run tardigrade_state_estimation vectornav_odometry
```

Converts VectorNav data into an odometry-style output.

```bash
ros2 run tardigrade_state_estimation zed_vectornav_odometry
```

Combines ZED and VectorNav inputs into `/tardigrade/state/odometry`.

The newer EKF path is configured in:

```text
src/tardigrade_bringup/config/zed_vectornav_ekf.yaml
```

It uses ZED visual odometry for position/linear velocity and VectorNav IMU data
for orientation/angular velocity. IMU linear acceleration is intentionally not
fused yet.

The EKF launch assumes the VectorNav is mounted FRD on the robot:

```text
X forward, Y right, Z down
```

ROS `base_link` remains FLU, so the launch publishes a default
`base_link -> vectornav` static transform with quaternion `(1, 0, 0, 0)`.

## Docker Helpers

```bash
./docker/run_jetson_hardware.sh
```

Fallback Jetson hardware container launcher when Docker Compose is unavailable.

## Container Shell Aliases

Interactive shells inside the container source `docker/ros_bashrc.sh`, which
defines a few short aliases:

```text
build-ws     /ws/build.sh
build-hw     /ws/build.sh --hardware
clean-build  /ws/build.sh --clean
mock         ros2 launch tardigrade_bringup mock.launch.py
status       ros2 topic echo /tardigrade/status
fg           ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```
