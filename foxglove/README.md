# Foxglove

Foxglove is the intended pool-test and bench-test UI for Tardigrade. Do not
start a custom dashboard unless Foxglove fails a specific requirement.

The first goal is a useful cockpit:

- camera images
- perception debug overlays
- ZED odometry
- VectorNav IMU
- `/tf`
- Pixhawk status
- command topics
- autonomy state once autonomy exists

## Rosbridge MVP

For ROS 2 Foxy, use `rosbridge_suite` first. It is older and easier to get
working on Foxy than `foxglove_bridge`.

Start rosbridge inside the container with:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

For local mock testing, start the mock PX4 stack and rosbridge together:

```bash
ros2 launch tardigrade_bringup mock_foxglove.launch.py
```

Then connect Foxglove using the Rosbridge connection option:

```text
ws://localhost:9090
```

On the Jetson, replace `localhost` with the Jetson's IP address.

When using `docker compose run`, service ports are published only if the
container is started with `--service-ports`. The repo launcher does this for
you:

```bash
./docker-build.sh
```

If the container was started another way, check `docker ps` on the host and
confirm it shows `9090->9090/tcp`.

## Foxglove Bridge Later

Foxglove's preferred bridge is `foxglove_bridge`, which opens a Foxglove
WebSocket on port `8765`.

ROS 2 Foxy does not provide the apt package `ros-foxy-foxglove-bridge` in the
standard ROS package index, so the bridge is not installed by the Dockerfile
yet. The source-build path has also pulled in missing dependencies, so treat it
as a later upgrade after the rosbridge layout is useful.

If `foxglove_bridge` is built from source or otherwise installed, start it with:

```bash
ros2 launch tardigrade_bringup foxglove_bridge.launch.py
```

Then connect Foxglove using the Foxglove WebSocket connection option:

```text
ws://localhost:8765
```

## Foxy Source Build

Tried and unavailable:

```bash
apt-get install ros-foxy-foxglove-bridge
```

Foxglove's ROS source tree is a separate ROS workspace inside the SDK repo. To
try it inside the Tardigrade container:

```bash
cd /tmp
git clone https://github.com/foxglove/foxglove-sdk
cd foxglove-sdk/ros
make
source install/setup.bash
```

Then return to the Tardigrade workspace and source it too:

```bash
cd /ws
source install/setup.bash
ros2 launch tardigrade_bringup foxglove_bridge.launch.py
```

If this works, decide whether to vendor the bridge as a submodule/source
dependency or keep it as a documented external setup step.

Open items:

1. Confirm whether current upstream `foxglove-sdk/ros` builds against ROS 2
   Foxy in this container.
2. If current upstream does not build, pin the newest Foxy-compatible bridge
   source version.
3. Add the bridge to the workspace as a source dependency or submodule only
   after the build path is proven.
4. Keep the Foxglove WebSocket port and launch command standardized on `8765`.

## Layouts

Committed layouts should live in this directory once the team has a first
working layout from Foxglove Studio.

Suggested files:

```text
foxglove/layouts/pool_test.json
foxglove/layouts/bench_debug.json
```

Do not spend time on custom Foxglove extensions yet. Start with standard panels:

- Image
- 3D
- Plot
- Raw Messages
- State Transitions
- Diagnostics-style status panels

## Topic Plan

See `foxglove/pool_test_topics.md` for the first set of topics the layout
should show.
