# Tardigrade ROS 2 Workspace

This repo is a ROS 2 Foxy workspace for the Tardigrade AUV. The normal
development path is Docker-based so laptops and the Jetson use the same ROS
toolchain.

The current ESP32 hardware path is:

```text
ZED + VectorNav -> EKF/state estimate -> /tardigrade/cmd_vel -> ESP32 PWM -> ESCs
```

The retired Pixhawk/PX4 path is preserved as legacy code, but `./build.sh`
skips it by default.

```text
src/legacy/px4_msgs       Legacy PX4 ROS 2 messages
src/legacy/tardigrade_px4 Legacy Pixhawk/PX4 code
src/legacy/COLCON_IGNORE  keeps legacy packages out of colcon builds
```

Most day-to-day work can happen locally without the ZED, VectorNav, ESP32, or
thrusters.

## Repo Guides

- `SETUP.md`: short setup instructions for local Docker and Jetson use.
- `SCRIPTS.md`: helper scripts, launch files, and ROS console scripts.
- `docs/`: lightweight runbooks and notes that should version with the code.

## Clone

Clone recursively so the vendor submodules are present:

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

Tracked external source layout:

```text
src/legacy/px4_msgs       Legacy PX4 ROS 2 messages; ignored by colcon
src/legacy/tardigrade_px4 Legacy Pixhawk/PX4 code; ignored by colcon
src/tardigrade_teleop     Keyboard/operator teleop tools
src/vectornav             VectorNav ROS 2 driver/messages
src/zed-ros2-wrapper      Stereolabs ZED wrapper pinned to humble-v4.0.8
  zed-ros2-interfaces     Nested submodule owned by zed-ros2-wrapper
```

Do not add a separate top-level `src/zed-ros2-interfaces`; the wrapper already
contains the matching nested interfaces package.

## Docker Setup

Build and start the development container:

```bash
./docker-build.sh --build
```

Start the development container after the image already exists:

```bash
./docker-build.sh
```

Inside the container:

```bash
cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./build.sh
source install/setup.bash
```

The base Compose file is safe for macOS and ordinary laptop development. It
mounts the repo at `/ws` and does not attempt to pass through Jetson hardware.

For Jetson hardware work, use the Jetson override from the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws \
  ./docker-build.sh --jetson
```

`WORKSPACE` is the host path mounted into the container at `/ws`. The Jetson
override adds host networking, privileged device access, `/dev`, USB, ZED SDK,
CUDA, and Tegra library mounts. ESP serial devices such as `/dev/ttyUSB*`,
`/dev/ttyACM*`, and `/dev/serial/by-id/*` are available inside the container
through the `/dev` mount. Do not use the Jetson override on a MacBook.

If Compose is unavailable on the Jetson, the fallback script remains:

```bash
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws ./docker/run_jetson_hardware.sh
```

## Local Development

Run the mock bringup first when working without hardware:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

For local Foxglove visualization, start rosbridge in another container shell:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Then connect Foxglove with the Rosbridge connection type:

```text
ws://localhost:9090
```

Useful local checks:

```bash
./build.sh
colcon test --packages-select tardigrade_interfaces tardigrade_state_estimation tardigrade_esp tardigrade_teleop tardigrade_bringup
```

The ZED wrapper source is present locally, but `zed_components`, `zed_wrapper`,
and `zed_ros2` require the Stereolabs ZED SDK. Build those only on the Jetson or
another machine with the ZED SDK installed/mounted.

If build output gets stale:

```bash
./build.sh --clean
```

## ESP Hardware Process

The short version of the Jetson/ESP flow is:

1. Start the Jetson hardware container.
2. Build and source the workspace inside `/ws`.
3. Start the ZED wrapper.
4. Start VectorNav.
5. Start `zed_vectornav_ekf.launch.py` and watch
   `/tardigrade/state/odometry/filtered`.
6. Start `foxglove_rosbridge.launch.py` if using Foxglove from a laptop.
7. Start `esp_thruster_bridge` with the ESP serial port and thruster map.
8. Start teleop or a controller that publishes `/tardigrade/cmd_vel`.
9. Keep thruster power disconnected until sensor, command, and mapping checks
   look sane.

The ESP runbook lives here:

```text
docs/esp_thruster_bringup.md
```

Core commands once sensors are running:

```bash
ros2 run tardigrade_esp esp_thruster_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p config_file:=/ws/config/esp_thruster_map.json
```

In another container terminal:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

## ESP32 Thruster Path

The ESP32 path is the active actuator path.

Flash:

```text
firmware/esp32_thruster_pwm/esp32_thruster_pwm.ino
```

Build and run inside the Jetson container:

```bash
colcon build --symlink-install --packages-select tardigrade_esp
source install/setup.bash
ros2 run tardigrade_esp esp_thruster_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p config_file:=/ws/config/esp_thruster_map.json
```

Then publish `/tardigrade/cmd_vel`, for example:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

The full procedure is documented in:

```text
docs/esp_thruster_bringup.md
```

The old Pixhawk/PX4 runbook is preserved for reference only:

```text
docs/jetson_zed_px4_startup.md
```
## Local Simulation Backend

For the VectorNav-only hardware controller and exact axis-by-axis test steps,
see [docs/vectornav_attitude_control.md](docs/vectornav_attitude_control.md).

Before Unity exists, run the ROS-only fake backend:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Then run shared mission logic against it:

```bash
ros2 run tardigrade_mission gate_mission
```

This is the contract Unity should later satisfy. Details live in:

```text
docs/local_sim_backend.md
docs/unity_simulation_plan.md
```

## Important Notes

- `tardigrade_esp` is the active ESP32 USB serial PWM path.
- `tardigrade_teleop` owns keyboard/operator velocity commands.
- `tardigrade_px4` and `px4_msgs` live under `src/legacy/` and are ignored by
  colcon.
- Unity simulation should use the same robot-level ROS contract as hardware.
  See `docs/unity_simulation_plan.md`.
- `.legacy_inspect` was stale git metadata and should not be restored.
- Do not command real thrust until `src/tardigrade_esp/config/esp_thruster_map.json` and
  `docs/thruster_mapping.md` have been verified against the physical vehicle.
