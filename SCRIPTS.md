# Scripts

Helper scripts and ROS entry points are listed here so new contributors can
find the important commands without searching through package files.

## Workspace Helpers

```bash
./docker-build.sh
```

Starts the local development container.

```bash
./docker-build.sh --build
```

Builds the Docker image and starts the local development container.

```bash
./docker-build.sh --jetson
```

Starts the Jetson hardware container using the base Compose file plus the
Jetson override.

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

Starts the combined ZED + VectorNav odometry path.

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Starts rosbridge on port `9090` for Foxglove's Rosbridge connection.

```bash
ros2 launch tardigrade_bringup foxglove_bridge.launch.py
```

Starts `foxglove_bridge` on port `8765` if the bridge has been built from
source or otherwise installed.

## Pixhawk / Control

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface
```

Current hardware Pixhawk interface over USB MAVLink. This is the main path for
external control, arming, status, and velocity setpoints.

```bash
ros2 run tardigrade_px4 keyboard_cmd_vel
```

Publishes keyboard velocity commands for bench testing.

```bash
ros2 run tardigrade_px4 prequal_mission
```

Runs the starter pre-qualification mission. It defaults to a dry run.

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
