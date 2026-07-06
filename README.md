# Tardigrade ROS 2 Workspace

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

Terminal 2, convert ZED pose into robot odometry:

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
