# Scripts

Documentation for root scripts and common ROS commands.

Build and source the workspace before running ROS commands:

```bash
./build.sh
source install/setup.bash
```

## `docker-build.sh`

Builds and starts the Docker container.

```bash
./docker-build.sh
```

Starts the local development container. For interactive Compose runs, this uses
`--service-ports` so host tools can reach ports such as rosbridge `9090`.

```bash
./docker-build.sh --build
```

Builds the image, then starts the local development container.

```bash
./docker-build.sh --rebuild
```

Builds the image without cache, then starts the local development container.

```bash
./docker-build.sh --detached
```

Starts the local development container in the background.

```bash
./docker-build.sh --jetson
```

Starts the Jetson hardware container using `docker/compose.yaml` plus
`docker/compose.jetson.yaml`. The Jetson override uses host networking, so
`docker ps` will not show per-port mappings.

## `build.sh`

Builds the ROS workspace.

```bash
./build.sh
```

Builds the local development workspace. By default, it skips ZED SDK packages
that only build on the Jetson or another machine with the Stereolabs SDK
installed.

```bash
./build.sh --hardware
```

Builds all packages, including the ZED SDK packages.

```bash
./build.sh --pkg PACKAGE
```

Builds one package.

```bash
./build.sh --debug
```

Builds with `RelWithDebInfo`.

```bash
./build.sh --clean
```

Removes `build`, `install`, and `log`, then rebuilds.

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

Starts the experimental `robot_localization` EKF path. It reads
`/zed/zed_node/odom` and `/vectornav/imu`, then publishes
`/tardigrade/state/odometry/filtered`.

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Starts rosbridge on port `9090` for Foxglove's Rosbridge connection.

```bash
ros2 launch tardigrade_bringup esp_depth_hold.launch.py
```

Starts the depth/attitude controller plus ESP thruster bridge.

## ESP / Control

```bash
ros2 run tardigrade_esp esp_thruster_bridge
```

Converts `/tardigrade/cmd_vel` into ESP PWM commands.

Monitoring topics:

```text
/tardigrade/thrusters/pwm
/tardigrade/esp/status
```

```bash
ros2 run tardigrade_esp esp_thruster_test
```

Bench utility for testing one ESP thruster output at a time.

```bash
ros2 run tardigrade_esp depth_attitude_controller
```

Holds depth, roll, and pitch while passing manual planar commands through.

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Publishes keyboard velocity commands for bench testing.

## State Estimation

```bash
ros2 run tardigrade_state_estimation zed_odometry
```

Converts ZED pose into the robot odometry topic.

```bash
ros2 run tardigrade_state_estimation vectornav_odometry
```

Converts VectorNav data into odometry-style output.

```bash
ros2 run tardigrade_state_estimation zed_vectornav_odometry
```

Combines ZED and VectorNav inputs into `/tardigrade/state/odometry`.

The EKF path is configured in:

```text
src/tardigrade_bringup/config/zed_vectornav_ekf.yaml
```

## Simulation / Missions

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Starts a ROS-only fake backend for status, odometry, and perception topics.

```bash
ros2 run tardigrade_mission gate_mission
```

Runs the gate mission against the active backend.

## Container Shell Aliases

Interactive shells inside the container source `docker/ros_bashrc.sh`, which
defines:

```text
build-ws     /ws/build.sh
build-hw     /ws/build.sh --hardware
clean-build  /ws/build.sh --clean
mock         ros2 launch tardigrade_bringup mock.launch.py
status       ros2 topic echo /tardigrade/status
fg           ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```
