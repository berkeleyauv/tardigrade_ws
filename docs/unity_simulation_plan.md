# Unity Simulation Plan

This plan sets up Unity as a replaceable simulator backend for the same ROS
autonomy and control scripts that run on the real robot.

## Core Rule

Autonomy code should talk only to robot-level ROS interfaces:

```text
/tardigrade/status
/tardigrade/state/odometry
/tardigrade/perception/gate
/tardigrade/perception/slalom
/tardigrade/cmd_vel
/tardigrade/set_external_control
/tardigrade/set_armed
```

Unity and hardware should both satisfy that contract.

## Backend Shape

```text
Shared ROS mission/control code
  |
  | subscribes:
  |   /tardigrade/status
  |   /tardigrade/state/odometry
  |   /tardigrade/perception/gate
  |   /tardigrade/perception/slalom
  |
  | publishes:
  |   /tardigrade/cmd_vel
  |
  v
Backend selected by launch/config
```

### Unity Backend

Unity should publish:

```text
/tardigrade/status
/tardigrade/state/odometry
/tardigrade/perception/gate
/tardigrade/perception/slalom
```

Unity should subscribe:

```text
/tardigrade/cmd_vel
```

Unity can also expose simple service handlers, or a small ROS shim can provide
them:

```text
/tardigrade/set_external_control
/tardigrade/set_armed
```

### Hardware Backend

Hardware should publish:

```text
/tardigrade/status
/tardigrade/state/odometry
real perception topics using the same detection messages
```

Hardware should subscribe:

```text
/tardigrade/cmd_vel
```

The subscriber may be the Pixhawk bridge, ESP bridge, or a future controller.

## Interfaces Added For Simulation

Gate detection:

```text
tardigrade_interfaces/msg/GateDetection
/tardigrade/perception/gate
```

Slalom marker detection:

```text
tardigrade_interfaces/msg/SlalomMarkerDetection
/tardigrade/perception/slalom
```

These messages are intentionally simple so Unity can publish perfect detections
first, and real perception can publish estimated detections later.

## Recommended Repo Layout

Keep this ROS workspace as the source of truth for:

```text
interfaces
mission scripts
behavior trees
control nodes
hardware adapters
launch files
```

Create a separate Unity repository for:

```text
Unity project files
scenes
materials
models
water/camera effects
synthetic data assets
large binaries
```

The Unity repo should depend on the ROS message definitions in this repo, but
the Unity project itself should not live inside the ROS workspace.

## Unity Setup Steps Later

1. Create a separate Unity project repository, for example
   `tardigrade_unity_sim`.
2. Install Unity's ROS-TCP-Connector package.
3. Run the ROS-TCP-Endpoint package in the local ROS environment.
4. Generate C# message classes for `tardigrade_interfaces`.
5. Build a simple pool scene with:
   - robot rigidbody,
   - gate object,
   - three slalom markers,
   - simulated camera.
6. Add a Unity ROS bridge component that:
   - subscribes to `/tardigrade/cmd_vel`,
   - moves the robot,
   - publishes `/tardigrade/state/odometry`,
   - publishes `/tardigrade/status`,
   - publishes gate/slalom detections.
7. Run existing ROS scripts against Unity first.
8. Swap Unity publishers for real sensors/perception on the robot.

## Local Docker And Unity Bridge

Unity runs on the local machine. ROS runs in the local Docker container. The
container exposes port `10000`, which is the default ROS-TCP endpoint port.

Start the local ROS container with service ports enabled:

```powershell
cd C:\Users\elila\Desktop\tardigrade_ws
docker compose -f docker/compose.yaml run --rm --service-ports tardigrade
```

Inside the container, build this workspace:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  tardigrade_interfaces tardigrade_sim tardigrade_mission
source install/setup.bash
```

The ROS-TCP-Endpoint package is a separate ROS package from Unity. Clone or add
it to a local sim workspace, then build it with this workspace. Once available,
run:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

In Unity's ROS Connection settings:

```text
Protocol: ROS2
ROS IP Address: 127.0.0.1
Port: 10000
```

If Unity cannot connect, check Windows Firewall and confirm the container was
started with `--service-ports`.

## PID And Control In Simulation

You do not need serious PID tuning on day one.

Start with a simple Unity controller:

```text
cmd_vel -> smooth velocity target -> rigidbody velocity/force
```

Use this to test mission logic and perception geometry. This does not need to
match the robot perfectly.

Add PID only when you want to test closed-loop behaviors such as:

```text
hold depth
hold heading
align to gate
track a slalom offset
reject drift/current
```

When you add PID, keep the same split as hardware:

```text
mission asks for motion
controller tries to achieve motion
sim/hardware applies physics
```

Do not tune the simulator so hard that autonomy only works in the simulator.
Use approximate drag, latency, saturation, and noise so the mission code stays
honest.
