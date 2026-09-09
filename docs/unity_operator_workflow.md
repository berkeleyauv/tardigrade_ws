# Unity operator workflow

This is the shortest repeatable path from the Unity editor to Foxglove and
keyboard teleop. It starts no mission or perception nodes.

## 1. Open Unity

Open `../tardigrade_unity_world` with Unity `6000.2.6f2`, open
`Assets/Scenes/SampleScene.unity`, but leave Play mode stopped for now. The
ROS-TCP Connector must be set to ROS 2, host `127.0.0.1`, and port `10000`.

Starting Play mode first also works: Unity remains safely disarmed and retries
the connection, but the Console will contain expected connection-refused noise.

## 2. Start and enter the Foxy container

From `tardigrade_ws` on the host:

```bash
./docker-build.sh --detached
docker compose -f docker/compose.yaml exec tardigrade bash
```

Use `./docker-build.sh --build --detached` the first time or after changing the
Docker image. Inside the container, build after pulling or editing ROS code:

```bash
build-ws
```

## 3. Start the operator stack

In the same container shell:

```bash
ros2 launch tardigrade_bringup unity_operator.launch.py
```

Images rebuilt from the current Dockerfile also provide the shorter alias:

```bash
unity-op
```

This starts:

- ROS-TCP Endpoint on host port `10000`;
- manual-source mux, velocity PID, wrench allocator, and actuator mapper;
- simulated IMU/VIO EKF and fixed robot transforms;
- Rosbridge on host port `9090`.

It does not start the gate detector, a mission, the ESP bridge, or any legacy
command adapter.

Return to Unity and press Play after the endpoint reports:

```text
Starting server on 0.0.0.0:10000
```

## 4. Verify the connection

Open another container shell:

```bash
docker compose -f docker/compose.yaml exec tardigrade bash
```

Then check each stream for a few seconds (Foxy's `ros2 topic echo` does not
support `--once`):

```bash
ros2 topic hz /clock
ros2 topic hz /tardigrade/sensors/imu/data
ros2 topic hz /tardigrade/sensors/visual_odometry
timeout 3 ros2 topic echo /tardigrade/status
timeout 3 ros2 topic echo /tardigrade/state/odometry/filtered
```

Expected results are a monotonic 100 Hz `/clock`, live sensor messages, status
with `control_connected: true`, and filtered odometry. Do not arm if `/clock`
or filtered odometry is absent.

The important command path is:

```text
/tardigrade/control/velocity_setpoint/manual
  -> /tardigrade/control/velocity_setpoint
  -> /tardigrade/control/wrench_command
  -> /tardigrade/actuators/thruster_forces
  -> /tardigrade/actuators/thruster_commands
  -> Unity
```

## 5. Open Foxglove

Create a **Rosbridge** connection to `ws://localhost:9090`. Import the JSON
files under `foxglove/layouts/`, starting with `sim_operator.json`. Use
`pid_tuning.json` for controller work and `sim_sensors.json` for estimator
and sensor checks.

In the operator layout:

1. Call reset with the `clean` scenario and a recorded seed.
2. In Vehicle Status, enable external control.
3. Use the two-click arm action.
4. Confirm link, controller, command, odometry, and allocation health before
   commanding motion.

Reset deliberately turns both safety gates off, so arm and external control
must be enabled again after every reset.

## 6. Run keyboard teleop

The operator launch already selects the `manual` mux source. In the second
interactive container shell run:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Keys are `w/s` surge, `j/l` sway, `r/f` heave, `a/d` yaw, and Space to stop.
Each key creates a 0.25 second command pulse, publishes its enable heartbeat,
then automatically sends zero. Tap repeatedly for continued movement.

Watch these topics while testing:

```bash
ros2 topic echo /tardigrade/control/velocity_setpoint/manual
ros2 topic echo /tardigrade/control/wrench_command
ros2 topic echo /tardigrade/actuators/thruster_commands
```

If the manual topic changes but wrench remains zero, check
`/tardigrade/control/command_fresh`, `/tardigrade/control/odometry_fresh`, and
that the operator launch is using `active_source:=manual`. If commands change
but Unity does not move, check the status message's `armed` and
`external_control_enabled` fields.

## 7. Stop safely

Release the keys, press Space, and call disarm in Foxglove. Stop the launch with
Ctrl-C, then stop Play mode in Unity. When finished with Docker:

```bash
docker compose -f docker/compose.yaml down
```
