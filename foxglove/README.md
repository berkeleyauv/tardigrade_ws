# Foxglove

Foxglove is the intended pool-test and bench-test UI for Tardigrade.

The first goal is a useful cockpit:

- camera images
- perception debug overlays
- ZED odometry
- VectorNav IMU
- `/tf`
- ESP/control status
- command topics
- autonomy state once autonomy exists

## Rosbridge MVP

For ROS 2 Foxy, use `rosbridge_suite` for the pool test. The committed PID
layout uses Service Call panels for tuning, so it does not depend on rosbridge
supporting Foxglove's native Parameters panel.

For phased checkout, import `layouts/pool_checkout.json`. Its individual
thruster panel calls a bounded ROS service rather than publishing an arbitrary
motor array: one slot at a time, at most 10%, for at most two seconds, followed
automatically by neutral.

Start rosbridge inside the container with:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
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

On Jetson hardware, the Compose override uses host networking. In that mode,
`docker ps` will not show a `9090->9090/tcp` port mapping. Connect to the
Jetson directly:

```text
ws://JETSON_IP:9090
```

## Launching Data Sources

Keep robot data sources and visualization separate. Start and stop rosbridge
without restarting sensors or state estimation.

For local mock topics:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

For Jetson ZED + VectorNav odometry, start the ZED wrapper first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
```

Then start VectorNav plus the combined odometry node:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
```

That publishes:

```text
/tardigrade/state/odometry
```

The current combined estimate is simple:

```text
position          ZED
orientation       VectorNav when fresh, otherwise ZED fallback
angular velocity  VectorNav
linear velocity   not estimated
```

There is also an experimental `robot_localization` EKF path:

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

It reads `/zed/zed_node/odom` and the converted
`/tardigrade/sensors/imu`, then publishes:

```text
/tardigrade/state/odometry/filtered
```

Use this filtered topic for comparison in Foxglove before replacing the main
`/tardigrade/state/odometry` path.

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

starts the visualization bridge.

## 3D Frames And Odometry

Foxglove's 3D frame dropdown comes from `/tf` and `/tf_static`, not directly
from every topic. The EKF launch publishes `odom -> base_link`; it also
publishes the static `base_link -> zed_camera_link` and
`base_link -> vectornav` mounting transforms. Run the ZED wrapper with
`publish_tf:=false` so the EKF remains the sole owner of the moving robot TF.

The simpler `/tardigrade/state/odometry` comparison topic does not publish TF.
Use `/tardigrade/state/odometry/filtered` plus the EKF TF tree for 3D display.

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
foxglove/layouts/zed.json
foxglove/layouts/state_estimation.json
```

`layouts/pid_tuning.json` is the pool-test tuning cockpit. It uses:

- `/tardigrade/control/set_pid_gains`
- `/tardigrade/control/set_axes_enabled`
- the four `/tardigrade/control/<axis>/debug` topics

See `docs/pool_teleop.md` for the mandatory safety gates and recording
commands. Live values are session scratch state; accepted gains must be copied
back into `src/tardigrade_esp/config/controller_gains.yaml`.

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
