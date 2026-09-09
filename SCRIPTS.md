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

Starts the `robot_localization` EKF path. It reads `/zed/zed_node/odom` and the
frame-corrected `/tardigrade/sensors/imu`, then publishes
`/tardigrade/state/odometry/filtered`.

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Starts rosbridge on port `9090` for Foxglove's Rosbridge connection.

```bash
ros2 launch tardigrade_bringup unity_operator.launch.py
```

Starts the interactive Unity stack: ROS-TCP, manual-source modern control,
simulated-sensor EKF, TF, and rosbridge. It intentionally excludes perception,
missions, ESP hardware, and legacy bridges. The container alias is `unity-op`.
See `docs/unity_operator_workflow.md` for the complete editor-to-keyboard path.

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

Starts the Xbox setpoint mapper, shared physical-unit control and allocation
stack, and current binary-protocol ESP bridge. It expects Foxglove to publish
`/joy` by default.

## ESP / Control

```bash
ros2 run tardigrade_esp esp_bridge --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Forwards named `/tardigrade/actuators/thruster_commands` to the ESP's bounded
`SetMotor` interface and publishes ESP telemetry. This standalone command is
for diagnostics; hardware modes start their own bridge, so stop the standalone
process before launching one.

Monitoring topics:

```text
/tardigrade/actuators/thruster_commands
/tardigrade/esp/state
```

Use the bounded one-at-a-time checkout mode instead of the legacy raw serial
test executable:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py
```

```bash
ros2 launch tardigrade_control control_stack.launch.py \
  active_source:=manual
```

Runs the modern command mux, pose guidance, physical-unit velocity controller,
bounded allocator, and nonlinear actuator mapper. It does not start Unity or
the ESP hardware backend.

For a modern manual dry test, run the keyboard publisher in an interactive
terminal. Its short command pulse also drives the mux enable signal:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

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

## Container Shell Aliases

Interactive shells inside the container source `docker/ros_bashrc.sh`, which
defines:

```text
build-ws     /ws/build.sh
build-hw     /ws/build.sh --hardware
clean-build  /ws/build.sh --clean
status       ros2 topic echo /tardigrade/status
fg           ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
unity-op     ros2 launch tardigrade_bringup unity_operator.launch.py
```
