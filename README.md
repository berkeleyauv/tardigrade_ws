# Tardigrade ROS 2 Workspace

This repository contains the ROS 2 Foxy workspace for the Berkeley AUV
Tardigrade. Development is Docker-based so laptops and the Jetson use the same
ROS toolchain.

Most day-to-day work can happen locally without the ZED, VectorNav, ESP32, or
thrusters.

## Contents

- [System Flow](#system-flow)
- [Repository Guides](#repository-guides)
- [Clone](#clone)
- [Build Packages](#build-packages)
- [Run Code](#run-code)
- [Foxglove](#foxglove)
- [Foxglove Live Bring-Up (Real ESP)](#foxglove-live-bring-up-real-esp)
- [Testing](#testing)
- [Safety Notes](#safety-notes)

## System Flow

The current hardware direction is ESP-first:

```mermaid
flowchart LR
    ZED[ZED camera] --> State[State estimation]
    VN[VectorNav IMU] --> State
    State --> Odom[/tardigrade/state/odometry/]
    Teleop[Teleop or mission] --> Cmd[/tardigrade/cmd_vel/]
    Cmd --> ESPBridge[tardigrade_esp bridge]
    ESPBridge --> ESP32[ESP32 PWM firmware]
    ESP32 --> ESCs[ESCs and thrusters]
```

For local development, `tardigrade_sim` can replace hardware with fake status,
odometry, and perception topics.

## Repository Guides

- [SETUP.md](SETUP.md): set up the repo, Docker container, Jetson host, and
  Foxglove.
- [SCRIPTS.md](SCRIPTS.md): root helper scripts, launch files, and ROS console
  commands.
- [CONTRIBUTING.md](CONTRIBUTING.md): local checks and PR expectations.
- [robot/README.md](robot/README.md): robot-host setup, udev rules, and
  autostart files.
- [foxglove/README.md](foxglove/README.md): visualization setup and layout
  notes.
- [docs/](docs): detailed runbooks that should version with the code.

## Clone

Clone recursively so vendor submodules are present:

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

External source layout:

```text
src/vectornav             VectorNav ROS 2 driver/messages
src/zed-ros2-wrapper      Stereolabs ZED wrapper
  zed-ros2-interfaces     Nested submodule owned by zed-ros2-wrapper
src/ROS-TCP-Endpoint      Unity ROS-TCP endpoint
```

## Build Packages

Start the development container:

```bash
./docker-build.sh --build
```

Inside the container:

```bash
cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./build.sh
source install/setup.bash
```

`./build.sh` skips ZED SDK packages by default because those only build on the
Jetson or another machine with the Stereolabs SDK installed.

## Run Code

For local development without hardware:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

For ESP thruster bridge testing after the workspace is built:

```bash
ros2 run tardigrade_esp esp_thruster_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p config_file:=/ws/config/esp_thruster_map.json
```

In another terminal:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

The full Jetson/ESP/ZED procedure is documented in
[docs/esp_thruster_bringup.md](docs/esp_thruster_bringup.md).

## Foxglove

Foxglove is the bench and pool-test visualization UI.

Start rosbridge inside the container:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove with the Rosbridge connection type:

```text
ws://localhost:9090
```

On the Jetson, replace `localhost` with the Jetson IP address. See
[foxglove/README.md](foxglove/README.md) for layout notes and bridge details.

## Foxglove Live Bring-Up (Real ESP)

End-to-end steps to go from "nothing running" to live ESP telemetry, arming,
and per-thruster bench testing in Foxglove — the F1 telemetry bridge plus the
motor-test/arm subset of F2 from
[foxglove_integration.md](../tardigrade_firmware/docs/foxglove_integration.md).

### 1. Attach the ESP (Windows laptop only — skip on the Jetson)

The ESP shows up as a native serial device on the Jetson already. On a Windows
laptop it has to be forwarded into WSL2 first:

```bash
usbipd list
```

```bash
usbipd attach --wsl --busid <busid>
```

(Run this in an elevated/Administrator PowerShell. `<busid>` comes from the
`usbipd list` output — look for the CP210x/CH340/FTDI USB-serial device.)

### 2. Start the container

Laptop bench (maps the ESP's serial device in via `docker/compose.esp.yaml`):

```bash
./docker-build.sh --esp --detached
```

Jetson (full `/dev` passthrough already, see `docker/compose.jetson.yaml` —
don't combine with `--esp`):

```bash
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws ./docker-build.sh --jetson --detached
```

`--detached` matters here: without it the container is tied to your current
shell/SSH session and dies if that session drops.

### 3. Find the ESP's port and build

```bash
docker exec -it tardigrade-foxy bash
```

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

Match the device against its `by-id` name (e.g. `CP2102` / `CP210x` is the
ESP32; an `FTDI` adapter is more likely the VectorNav) — port numbers can
swap between reboots, so don't assume `ttyUSB0` is the ESP.

```bash
./build.sh --pkg tardigrade_esp
source install/setup.bash
```

(First build on a fresh checkout: run `./build.sh` with no `--pkg` instead, so
`tardigrade_interfaces` and other dependencies build too.)

### 4. Run the bridge

```bash
ros2 run tardigrade_esp esp_bridge --ros-args -p serial_port:=/dev/ttyUSB1
```

Leave this running. Before starting a second one anywhere, check
`ros2 node list` — two `esp_bridge` processes fighting over the same serial
port fails with "device disconnected or multiple access on port."

### 5. Start rosbridge

New shell, same container:

```bash
docker exec -it tardigrade-foxy bash
```

```bash
source install/setup.bash
fg
```

Leave this running too (`fg` is the `docker/ros_bashrc.sh` alias for
`ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py`).

### 6. Connect Foxglove

Open [app.foxglove.dev](https://app.foxglove.dev) (or Foxglove Desktop) →
**Open connection** → **Rosbridge (ROS 1 & 2)** →

```text
ws://localhost:9090          # same laptop
ws://<jetson-ip>:9090        # Jetson — get the IP from `ip addr` on the Jetson
```

Add a **Raw Messages** panel on `/tardigrade/esp/state` to confirm live
telemetry (not stale/fake data — `fake_esp_state` and `esp_bridge` publish the
same topic, so don't run both at once).

**Multiple laptops connecting at once** (F3) only works if they're on a
network that actually routes between them — a laptop-to-Jetson USB
tether/Windows Mobile Hotspot (`192.168.137.x`) only reaches the one laptop
providing it. Use a phone hotspot or a dedicated router instead, and watch out
for client/AP isolation on venue Wi-Fi, which silently blocks device-to-device
traffic even on the same network.

### 7. Arm and test a thruster

Add a **Service Call** panel: service name `/tardigrade/set_armed`, request
`{"armed": true}` → **Call service** → confirm `armed` flips to `true` in the
state panel.

**Kill-switch duty and clear thrusters before this step** — see
[foxglove_integration.md's safety model](../tardigrade_firmware/docs/foxglove_integration.md#safety-model-physical-kill-switch-not-a-software-presence-beacon).

Add a **Publish** panel on `/tardigrade/thrusters/cmd`
(`std_msgs/Float32MultiArray`). Edit the `data` array — one index nonzero,
rest zero:

```json
"data": [0, 0, 0, 0.15, 0, 0, 0, 0]
```

**Publish**, observe, then zero it and publish again immediately. Index
position = thruster slot, see
[docs/thruster_mapping.md](docs/thruster_mapping.md). The firmware clamps
bench motor-test throttle to ±0.30 regardless of what's sent
(`Safety::commandMotor`'s `test_limit_`), so this can't overdrive a thruster.

Disarm when done: same Service Call panel, `{"armed": false}`.

## Testing

Useful local checks:

```bash
./build.sh
colcon test --packages-select \
  tardigrade_interfaces \
  tardigrade_state_estimation \
  tardigrade_esp \
  tardigrade_teleop \
  tardigrade_bringup \
  tardigrade_mission \
  tardigrade_sim
colcon test-result --verbose
```

GitHub Actions runs the Docker build and active package tests on PRs and pushes
to `main`.

## Safety Notes

- Keep thruster power disconnected until sensor, command, and mapping checks
  look sane.
- Verify [config/esp_thruster_map.json](config/esp_thruster_map.json) and
  [docs/thruster_mapping.md](docs/thruster_mapping.md) against the physical
  vehicle before commanding real thrust.
- Keep arming and external-control enable explicit; do not hide them inside
  launch files.
