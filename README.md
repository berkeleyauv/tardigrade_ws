# Tardigrade ROS 2 Workspace

This repo is a ROS 2 Foxy workspace for the Tardigrade AUV. The normal
development path is Docker-based so laptops and the Jetson use the same ROS
toolchain.

The current hardware path is:

```text
ZED pose + VectorNav IMU -> /tardigrade/state/odometry -> USB MAVLink -> Pixhawk/PX4
```

Most day-to-day work can happen locally without the ZED, VectorNav, or Pixhawk.
The Jetson/Pixhawk arming procedure is documented as a runbook because it has
hardware-specific requirements and failure modes.

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
src/px4_msgs              PX4 ROS 2 messages for legacy/mock uXRCE paths
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
override adds host networking, privileged device access, USB, ZED SDK, CUDA,
and Tegra library mounts. Do not use the Jetson override on a MacBook.

If Compose is unavailable on the Jetson, the fallback script remains:

```bash
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws ./docker/run_jetson_hardware.sh
```

## Local Development

Run the mock bringup first when working without hardware:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

Useful local checks:

```bash
./build.sh
colcon test --packages-select tardigrade_interfaces tardigrade_state_estimation tardigrade_px4 tardigrade_bringup
```

The ZED wrapper source is present locally, but `zed_components`, `zed_wrapper`,
and `zed_ros2` require the Stereolabs ZED SDK. Build those only on the Jetson or
another machine with the ZED SDK installed/mounted.

If build output gets stale:

```bash
./build.sh --clean
```

## Pixhawk Arming Process

The short version of the Jetson/Pixhawk arming flow is:

1. Start the Jetson hardware container.
2. Build and source the workspace inside `/ws`.
3. Start the ZED wrapper.
4. Start `zed_vectornav_state.launch.py` to publish `/tardigrade/state/odometry`.
5. Start `mavlink_pixhawk_interface` on `/dev/ttyACM0`.
6. Confirm `/tardigrade/status` shows PX4 connected, fresh visual odometry, and
   PX4 local position.
7. Enable external control/Offboard.
8. Arm.
9. Disarm before touching hardware.

The detailed procedure, terminal layout, expected logs, and troubleshooting
checks live here:

```text
docs/jetson_zed_px4_startup.md
```

Core commands once the sensors are running:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

In another container terminal:

```bash
ros2 topic echo /tardigrade/status
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Important: a ROS service response of `success: true` only means the Jetson sent
the MAVLink command. PX4 acceptance is shown by MAVLink command ACKs in
`/tardigrade/status.detail` and in the Pixhawk interface logs.

Useful ACKs:

```text
176/0  Offboard mode command accepted
400/0  arm/disarm command accepted
400/1  arm temporarily rejected
400/2  arm denied
```

## Pre-Qualification Mission

The starter pre-qualification script is:

```bash
ros2 run tardigrade_px4 prequal_mission
```

By default this is a dry run: it checks Pixhawk status and odometry but does
not arm or publish motion. The real run uses `dry_run:=false` after the ZED,
odometry bridge, and `mavlink_pixhawk_interface` are already running in
velocity mode.

The real run can include `target_depth_m`, which is positive downward from the
robot's starting odometry `z`; the mission descends to that depth and holds it
throughout the path.

Detailed commands live in:

```text
docs/jetson_zed_px4_startup.md
```

## Local Simulation Backend

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

- `mavlink_pixhawk_interface` is the current hardware Pixhawk path.
- `pixhawk_interface` is the older PX4 ROS 2 `/fmu/*` path for mock/uXRCE work.
- `px4_msgs` is not required for the MAVLink hardware path.
- Unity simulation should use the same robot-level ROS contract as hardware.
  See `docs/unity_simulation_plan.md`.
- `behavior_trees/pathing_mission.xml` is a Groot/BehaviorTree.CPP pathing
  design artifact. Its node contracts are described in
  `docs/pathing_behavior_tree.md`.
- `.legacy_inspect` was stale git metadata and should not be restored.
- Do not command real thrust until `config/thruster_map.yaml` and
  `docs/thruster_mapping.md` have been verified against the physical vehicle.
