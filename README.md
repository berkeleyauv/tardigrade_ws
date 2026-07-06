# Tardigrade ROS 2 Workspace

![Welcome](lol/hehe.jpg)

This workspace is set up for ROS 2 Foxy development in Docker. The container
keeps every teammate on the same Ubuntu/ROS toolchain while mounting this repo
into `/ws`, so code edits stay on the host and builds happen inside Linux.

## Setup

### Cloning
Since there are submodules, clone recursively
```
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git 
```

If the repo was already cloned without submodules, run:

```
git submodule update --init --recursive
```

### Docker

Build the image:

```
docker build -t tardigrade-foxy -f docker/Dockerfile .
```

Run an interactive container from the repo root:

```
docker run -it --rm --network host --ipc host --privileged -v "$(pwd):/ws" -v /dev:/dev tardigrade-foxy
```

Or use Docker Compose:

```
docker compose -f docker/compose.yaml up --build -d
docker exec -it tardigrade-foxy bash
```

The compose file is intended for Linux or the Jetson. Docker Desktop on Windows
does not expose robot serial devices the same way and may not support host
networking the way ROS 2 expects.

On older Docker installs, the compose command may be:

```
docker-compose -f docker/compose.yaml up --build -d
docker exec -it tardigrade-foxy bash
```

### Building the ROS workspace

Inside the container:

```
cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

After that, new shells in the container automatically source ROS Foxy and
`/ws/install/setup.bash` if it exists.

### Running the mock bringup

Use this first because it does not require the VectorNav IMU or Pixhawk:

```
ros2 launch tardigrade_bringup mock.launch.py
```

### Running with the VectorNav IMU

Plug in the VectorNav and check which serial device appeared on the Jetson:

```
ls -l /dev/ttyUSB* /dev/ttyACM*
dmesg | tail
```

Then launch with the matching port:

```
ros2 launch tardigrade_bringup vectornav_state.launch.py port:=/dev/ttyUSB1 baud:=115200
```

The compose setup runs the container with `/dev` mounted and `privileged: true`,
which is the simple development mode for serial hardware. For a tighter
competition deployment, replace that with explicit `--device=/dev/ttyUSBx`
arguments once the port names are stable.

### How the Docker setup works

- `docker/Dockerfile` defines the image: ROS 2 Foxy plus build tools such as
  `colcon`, `rosdep`, `vcstool`, and basic terminal utilities.
- `docker/ros_entrypoint.sh` runs every time the container starts. It sources
  ROS Foxy, then sources `/ws/install/setup.bash` if the workspace has already
  been built.
- `docker/compose.yaml` is a reusable version of the long `docker run` command.
  It mounts this repo at `/ws`, uses host networking for ROS 2 discovery, and
  exposes serial devices under `/dev`.
- The image is mostly the operating environment. Your source code is not baked
  into it during development because the workspace is bind-mounted into the
  container.

### Jetson ZED + Pixhawk arming

For the current Jetson + ZED + Pixhawk arming workflow, use the full runbook:

```
docs/jetson_zed_px4_startup.md
```

The short version is:

1. Start the Jetson/ZED container from the Jetson host:

```
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws bash ./docker/run_jetson_zed.sh
```

2. In the container, build/source the ROS workspace:

```
cd /ws
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

3. Run these in separate container terminals. Open additional terminals with:

```
sudo docker exec -it tardigrade-foxy bash
cd /ws
source install/setup.bash
```

Terminal 1, ZED camera:

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Terminal 2, start VectorNav and convert ZED + VectorNav into robot odometry:

```
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
```

If the VectorNav is not connected and you only need the ZED pose path, use:

```
ros2 launch tardigrade_bringup zed_state.launch.py
```

Terminal 3, connect to Pixhawk over USB MAVLink and send visual odometry:

```
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

Terminal 4, arm from ROS:

```
ros2 topic echo /tardigrade/status
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 topic echo /tardigrade/status
```

Success looks like:

```
px4_connected: true
armed: true
external_control_enabled: true
```

Important: `configure_px4_params:=true` is currently required because the
Pixhawk parameter save path is not working on the bench board. The node sends
the required PX4 params into RAM at startup, so they are lost when the Pixhawk
reboots.

ROS interfaces used by this flow:

```
/zed/zed_node/pose
```

Pose from the ZED wrapper. This is the robot's local position source.

```
/vectornav/imu
```

IMU data from the VectorNav. This is used for attitude and angular velocity.

```
/tardigrade/state/odometry
```

Fused robot state published by `zed_vectornav_odometry`. The Pixhawk interface
sends this to PX4 as MAVLink visual odometry.

```
/tardigrade/status
```

Robot/Pixhawk status published by `mavlink_pixhawk_interface`. Use this to
confirm `px4_connected`, `armed`, `external_control_enabled`, visual odometry
freshness, and command acknowledgements.

```
/tardigrade/cmd_vel
```

Teleop velocity command. It uses `geometry_msgs/Twist` in ROS body-frame FLU:
`linear.x` forward, `linear.y` left, `linear.z` up, and `angular.z` yaw left.

```
/tardigrade/set_external_control
```

Service that switches PX4 into Offboard/external-control mode and starts the
setpoint stream.

```
/tardigrade/set_armed
```

Service that sends the MAVLink arm/disarm command to PX4.

### Safe Teleop Dry Run

For a no-motion teleop dry run, restart the Pixhawk interface in velocity mode
without speed limits. This node subscribes to `/tardigrade/cmd_vel`, but the
defaults clamp all commanded motion to zero:

```
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity
```

Then publish keyboard velocity commands from another container terminal. This
node only publishes desired motion; it does not talk to the Pixhawk directly:

```
ros2 run tardigrade_px4 keyboard_cmd_vel
```

Keyboard controls:

```
w/s     forward/back
j/l     strafe left/right
r/f     up/down
a/d     yaw left/right
space   zero command
Ctrl-C  quit
```

In the Pixhawk interface terminal, look for:

```
cmd_vel_received=...
```

That proves the keyboard node is reaching the Pixhawk interface. With all speed
limits at zero, the Pixhawk can be armed but should command no motion.

Arm in another container terminal:

```
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 topic echo /tardigrade/status
```

Disarm:

```
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Only after a dry run should nonzero velocity clamps be added, one axis at a
time, with thrusters made safe:

```
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

Before commanding real thrust, fill out `config/thruster_map.yaml` and follow
`docs/thruster_mapping.md`. The robot does not need to match the old diagram,
but the physical wiring, PX4 actuator setup, and documented force directions
must match.

Test only one axis at a time at first. If the wrong thrusters move, fix the PX4
actuator/mixer/output configuration before increasing limits. The ROS code
commands body motion; PX4 maps that motion to the physical thrusters.

Build this image on the Jetson when you intend to run on the Jetson:

```
docker compose -f docker/compose.yaml build
```

The Jetson AGX Xavier is ARM64, while most laptops are AMD64. A Docker image
built on one architecture will not normally run on the other unless you use
multi-architecture builds. For this robotics workflow, the simplest rule is:
build on the machine that will run the robot.

If you need GPU access later, install NVIDIA's Docker runtime on the Jetson and
add the NVIDIA runtime/device settings to the compose file. The current ROS
nodes in this repo do not appear to require CUDA.

### Pixhawk/PX4 note

There are two PX4 communication paths in the workspace:

- `pixhawk_interface` uses PX4 ROS 2 `/fmu/...` topics and requires a PX4 ROS 2
  bridge such as uXRCE-DDS.
- `mavlink_pixhawk_interface` talks directly to the Pixhawk over USB MAVLink,
  usually `/dev/ttyACM0`. This is the path used by the current bench arming
  workflow.

### Cleaning build outputs

If generated files get stale, remove the ROS build outputs from the repo root:

```
rm -rf build install log
```
