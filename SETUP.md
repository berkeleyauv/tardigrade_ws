# Setup

This is the short setup path for the Tardigrade ROS 2 workspace. The detailed
Jetson/Pixhawk/ZED runbook lives in `docs/jetson_zed_px4_startup.md`.

## Clone

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Docker Development

Build and start the dev container:

```bash
./docker-build.sh --build
```

Start the dev container after the image already exists:

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

The skipped ZED packages require the Stereolabs ZED SDK. Build them on the
Jetson or another machine with the ZED SDK installed.

## Jetson Hardware Container

From the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws \
  ./docker-build.sh --jetson
```

The Jetson override enables host networking, device access, CUDA/Tegra mounts,
and the ZED SDK path. Do not use it for normal laptop development.

## Local Mock Bringup

After building and sourcing:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

## Common Checks

```bash
colcon test --packages-select tardigrade_interfaces tardigrade_state_estimation tardigrade_px4 tardigrade_bringup
```

If build state gets stale:

```bash
./build.sh --clean
source install/setup.bash
```
