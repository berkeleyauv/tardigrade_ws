# Local Sim Backend

This is a ROS-only stand-in for the future Unity scene. It lets us test mission
logic before Unity exists.

## What It Provides

Run:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

It provides:

```text
/tardigrade/status
/tardigrade/state/odometry
/tardigrade/perception/gate
/tardigrade/perception/slalom
/tardigrade/set_external_control
/tardigrade/set_armed
```

It subscribes:

```text
/tardigrade/cmd_vel
```

When armed and external control is enabled, it integrates `/tardigrade/cmd_vel`
into fake odometry. It also publishes fake gate and slalom detections.

## Build

From the local Docker container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  ros_tcp_endpoint tardigrade_interfaces tardigrade_sim tardigrade_mission
source install/setup.bash
```

## Test The Backend

Terminal 1:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Terminal 2:

```bash
ros2 topic echo /tardigrade/status
ros2 topic echo /tardigrade/state/odometry
ros2 topic echo /tardigrade/perception/gate
```

Terminal 3:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

The fake robot moves only after these services are called:

```bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

## Test The Gate Mission

Terminal 1:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Terminal 2:

```bash
ros2 run tardigrade_mission gate_mission
```

The mission:

1. waits for fake status,
2. enables external control,
3. arms the fake backend,
4. spins/searches until the gate is visible,
5. aligns to the gate yaw error,
6. drives forward through the gate,
7. stops, disarms, and disables external control.

Unity now replaces `fake_unity_backend` in the same way: `RosRobotBridge.cs`
in `tardigrade_unity_sim` (see
[the Unity sim README](../../tardigrade_unity_sim/README.md)) implements the
same topics and services against the robot GameObject in the pool scene. Any
mission or path script below runs unmodified against either backend — swap
which backend node/editor session is running, nothing else changes.

## Driving A Custom Path

`tardigrade_mission` ships a `path_follower` node for waypoint-following
without writing a full mission. It is backend-agnostic: it only reads
`/tardigrade/state/odometry` and writes `/tardigrade/cmd_vel`, so it works
identically against `fake_unity_backend` or the Unity `RosRobotBridge`.

Terminal 1 (backend, either one):

```bash
ros2 run tardigrade_sim fake_unity_backend
# or: press Play in tardigrade_unity_sim with ros_tcp_endpoint running
```

Terminal 2:

```bash
ros2 run tardigrade_mission path_follower
```

By default it drives a 3 m square. Waypoints, speed, and tolerances are all
ROS parameters, so you do not need to edit the script to change the path:

```bash
ros2 run tardigrade_mission path_follower --ros-args \
  -p waypoints:="[2.0, 0.0, 2.0, 2.0, 0.0, 2.0]" \
  -p forward_cmd:=0.4 \
  -p target_depth_m:=1.0
```

`waypoints` is a flat `[x0, y0, x1, y1, ...]` list in the odom frame (metres,
robot's start position is the origin). Useful parameters:

| Parameter | Meaning |
|---|---|
| `waypoints` | Flat x,y list to drive through in order. |
| `target_depth_m` | Depth to hold while driving (0 = ignore z axis). |
| `forward_cmd` / `yaw_kp` | Speed and turn-rate aggressiveness (normalized `cmd_vel` units, same scale as `fake_unity_backend`/`RosRobotBridge`). |
| `heading_align_threshold_rad` | How square-on the robot must face a waypoint before driving forward, vs. turning in place. |
| `position_tolerance_m` / `depth_tolerance_m` | "Close enough" radius to call a waypoint reached. |

Or set these as defaults in a launch file / YAML params file instead of
passing `--ros-args` every time, the same way other `tardigrade_*` nodes do.

### Does This Need A Rebuild?

The workspace is built with `colcon build --symlink-install` (see
`build.sh`), so Python nodes run directly from `src/`, not from a copied
`install/` tree. What that means in practice:

| Change | Rebuild needed? |
|---|---|
| Different `waypoints` / other params via `--ros-args` or a params YAML | **No.** Just re-run `ros2 run ...` with the new args. |
| Editing logic inside an existing file, e.g. `path_follower.py` | **No**, as long as you already built the package once. The symlink picks up the edit immediately — just re-run the node. |
| Adding a **new** script/node file to a package | **Yes.** New files need an entry in `setup.py`'s `console_scripts` (see below), and colcon has to regenerate that wrapper: `./build.sh --pkg tardigrade_mission`. |
| Editing a `.msg`/`.srv` in `tardigrade_interfaces` | **Yes**, interface packages are compiled: `./build.sh --pkg tardigrade_interfaces` (and regenerate the Unity C# messages too, see the Unity README). |

If in doubt, `./build.sh --pkg tardigrade_mission` is fast (a few seconds for
a pure-Python package) and always safe to re-run.

### Writing A Different Path Shape

If parameters aren't expressive enough (e.g. you want an arc, a timed hold at
a waypoint, or a reactive path), copy
`src/tardigrade_mission/tardigrade_mission/path_follower.py` as a starting
point. The two methods that matter:

- `go_to_waypoint(wx, wy)`: the per-waypoint control loop. It reads
  `current_pose()` (from the latest odometry) each tick and publishes a
  `Twist` on `/tardigrade/cmd_vel`. Replace the turn-then-drive logic here for
  a different steering behavior.
- `run()`: the top-level sequence — arms, iterates waypoints, disarms.
  Extend this for waypoint-specific behavior (e.g. pause, look for a gate)
  between legs.

Register any new script as a console entry point in
`src/tardigrade_mission/setup.py` under `console_scripts`, then rebuild with
`./build.sh --pkg tardigrade_mission` (required — see the rebuild table
above — new entry points don't exist until colcon regenerates them).
