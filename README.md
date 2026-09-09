# Tardigrade ROS 2 Workspace

ROS 2 Foxy software for the Berkeley AUV Tardigrade. Development and robot
operation use the same Docker workspace.

## Current Architecture

The Jetson owns sensing, state estimation, guidance, velocity control, and
geometry-based allocation. Unity and the ESP32 consume the same named,
normalized actuator command. The ESP32 remains an independent safety layer.

```mermaid
flowchart LR
    ZED[ZED camera] --> EKF[Jetson state estimation]
    VN[VectorNav IMU] --> EKF
    Xbox[Xbox on MacBook] --> Sources[Stamped command sources]
    Mission[Mission] --> Sources
    Pose[Pose target] --> Guidance[Pose guidance]
    Guidance --> Sources
    Sources --> Mux[Explicit source mux]
    EKF --> Control[Velocity/rate controller]
    Mux --> Control
    Control -->|Wrench, N and N m| Allocator[Geometry allocator]
    Allocator -->|Named forces, N| Mapper[Actuator curves]
    Mapper -->|Named commands| Unity[Unity plant]
    Mapper -->|Named commands| Bridge[ESP serial bridge]
    Bridge --> Safety[ESP arm, watchdog, authority cap]
    Safety --> ESC[ESCs and thrusters]
```

The Jetson does **not** forward pose to the ESP during normal operation. Do not
run the firmware repository's `pose_bridge.py`, `gcs_server.py --ros`, or the
ROS synthetic-pose test hook with the Jetson controller.

Package ownership is deliberately narrow:

| Package | Owns |
|---|---|
| `tardigrade_interfaces` | Shared ROS messages and services only |
| `tardigrade_description` | Canonical vehicle, actuator, and sensor configuration |
| `tardigrade_control` | Backend-agnostic controllers, allocation, and command mapping |
| `tardigrade_state_estimation` | Sensor conversion and filtered state |
| `tardigrade_esp` | ESP serial protocol and hardware actuator adapters only |
| `tardigrade_bringup` | Complete real-hardware and Unity launch profiles |
| `tardigrade_mission` | Preserved prequalification and qualification missions |

Complete operating modes are launched from `tardigrade_bringup`; library-like
packages may retain small component launch files, but they do not compose a
whole robot mode.

## Canonical Guides

- [Pool runbook](docs/pool_teleop.md): complete build, network, sensor,
  Foxglove, Xbox, thruster, teleop, recording, and PID procedure.
- [Runtime modes](docs/runtime_modes.md): short command reference.
- [Coordinate frames](docs/coordinate_frames.md): `map`, `odom`, `base_link`,
  sensor mounting, and sign checks.
- [Thruster mapping](docs/thruster_mapping.md): slots, pins, coefficients, and
  physical verification.
- [Jetson control architecture](docs/jetson_control_architecture.md): ownership
  and safety boundaries.
- [Unity SIL simulator](docs/unity_sil.md): physical model, ROS contract,
  deterministic scenarios, and configuration workflow.
- [Foxglove setup](foxglove/README.md): connection, layouts, and MacBook Xbox
  extension.
- [Development setup](SETUP.md), [scripts](SCRIPTS.md), and
  [robot host setup](robot/README.md).

## Clone And Build

Clone recursively so the VectorNav and ZED sources are present:

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

Local development container:

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

The default build skips hardware-only ZED SDK packages. On the Jetson use:

```bash
./build.sh --hardware
source install/setup.bash
```

After deleting or renaming launch files, use a clean hardware build so stale
installed launch files disappear:

```bash
./build.sh --clean --hardware
source install/setup.bash
```

## Common Commands

Realistic Unity software-in-the-loop stack:

```bash
ros2 launch tardigrade_bringup unity_sil.launch.py
```

Interactive Unity + Foxglove + keyboard workflow:

```bash
ros2 launch tardigrade_bringup unity_operator.launch.py
```

See [the Unity operator workflow](docs/unity_operator_workflow.md) for the full
startup, safety-gate, layout-import, and teleop sequence.

VectorNav only on the current robot:

```bash
ros2 launch tardigrade_bringup vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200
```

Foxglove transport:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Bounded individual-thruster checkout:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Assisted teleop after the state-estimation and individual-thruster gates pass:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

Only one load-bearing mode may run at once. `thruster_checkout_real`,
`pool_assisted`, `prequal_autonomy`, and `qual_autonomy` each start their own
ESP bridge.

## Stable Robot Ports

Current hardware identities:

```text
VectorNav  /dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0
ESP32      /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Confirm them after hardware changes:

```bash
ls -l /dev/serial/by-id/
```

Never rely on `/dev/ttyUSB0` versus `/dev/ttyUSB1`; those numbers can swap.

## Testing

```bash
./build.sh
colcon test --packages-select \
  tardigrade_interfaces \
  tardigrade_description \
  tardigrade_state_estimation \
  tardigrade_control \
  tardigrade_esp \
  tardigrade_teleop \
  tardigrade_bringup \
  tardigrade_mission
colcon test-result --verbose
```

Hardware operation additionally requires every go/no-go gate in the
[pool runbook](docs/pool_teleop.md).

## Safety

- Keep thruster power disconnected until command signs and stop behavior are
  verified.
- Keep one person assigned to the physical kill switch during powered tests.
- Use exactly one ESP serial owner and one thruster-command source.
- Release LB, disarm, and remove thruster power before troubleshooting.
- Never treat an estimator, joystick, network, or watchdog failure as something
  to work around; it blocks powered testing.
