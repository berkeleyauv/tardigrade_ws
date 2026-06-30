# Week-One Bringup Plan

## Goal

By the end of the week, Tardigrade should have a working bringup path for:

1. VectorNav-based state input.
2. PX4 communication through Micro XRCE-DDS.
3. Offboard heartbeat and PX4 command publishing.
4. Arm/disarm and external-control services.
5. Keyboard teleop through robot-level commands.

This is a bringup milestone, not an autonomy milestone. Gate autonomy comes after the robot can be safely started, monitored, armed, and commanded.

## Milestone 1: Build Core Dependencies

Build `px4_msgs` first.

Recommended command inside Docker:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select px4_msgs --parallel-workers 1 --event-handlers console_direct+
```

Notes:

- The first `px4_msgs` build is slow because it generates many ROS interfaces.
- Keep `px4_msgs` pinned to the PX4 firmware version on the Pixhawk.
- Current default is PX4 `v1.14.0`.

Add VectorNav from the `ros2` branch:

```bash
cd /ws
git submodule add -b ros2 https://github.com/berkeleyauv/vectornav.git src/vectornav
git submodule update --init --recursive
```

Then build:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select vectornav_msgs vectornav --parallel-workers 1 --event-handlers console_direct+
```

## Milestone 2: Define Robot Interfaces

Create `tardigrade_interfaces`.

Initial interfaces:

```text
srv/SetArmed.srv
bool armed
---
bool success
string message
```

```text
srv/SetExternalControl.srv
bool enabled
---
bool success
string message
```

Use standard messages unless a custom robot concept is needed:

- prefer `nav_msgs/Odometry` for robot odometry
- prefer `geometry_msgs/TwistStamped` for teleop velocity commands
- add a custom `RobotStatus.msg` for combined robot/PX4 health

Acceptance criteria:

- `tardigrade_interfaces` builds independently.
- No interface references `px4_msgs`.
- Other packages can depend on these interfaces without pulling in PX4 concepts.

## Milestone 3: Establish PX4 Communication

Run the Micro XRCE-DDS Agent in the environment connected to the Pixhawk.

Checklist:

- Pixhawk firmware matches the checked-out `px4_msgs` version.
- Agent is running before expecting `/fmu/*` topics.
- ROS domain and network settings are correct.
- `/fmu/out/vehicle_status` appears.
- `/fmu/in/vehicle_command` exists or can be published to.
- QoS settings are compatible with PX4 ROS 2 topics.

Useful checks:

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
```

Acceptance criteria:

- ROS 2 can see PX4 topics.
- `VehicleStatus` messages arrive when Pixhawk is connected.

## Milestone 4: PX4 Driver Services And Heartbeat

Update `tardigrade_px4` so it owns:

- Offboard heartbeat publishing
- arm/disarm service
- external-control enable service
- PX4 status subscription
- robot status publication

Safety behavior:

- no automatic arming
- no automatic external-control enable
- heartbeat can run continuously while the driver is active
- commands should log clearly when sent
- teleop setpoints should be ignored unless external control is enabled

Acceptance criteria:

- Calling the arm service publishes the correct PX4 `VehicleCommand`.
- Calling the disarm service publishes the correct PX4 `VehicleCommand`.
- Calling external-control enable publishes the Offboard mode command.
- `/tardigrade/status` updates from PX4 status.

## Milestone 5: VectorNav Bringup

Launch the VectorNav driver:

```bash
ros2 launch vectornav vectornav.launch.py
```

Expected topics:

- `/vectornav/raw/common`
- `/vectornav/imu`
- `/vectornav/pose`
- `/vectornav/velocity`

Useful checks:

```bash
ros2 topic echo /vectornav/imu
ros2 topic hz /vectornav/imu
```

Acceptance criteria:

- `/vectornav/imu` publishes `sensor_msgs/Imu`.
- Frame IDs are understood and documented.
- Serial port and baud rate are correct for the robot.

## Milestone 6: State Estimation Bridge

Create `tardigrade_state_estimation`.

Week-one scope:

- subscribe to VectorNav outputs
- validate message freshness
- publish robot odometry
- provide the PX4 driver with PX4-ready odometry input

Do not build a full custom EKF yet.

Acceptance criteria:

- State estimation publishes a stable robot odometry topic.
- The PX4 driver can convert that odometry into the PX4 odometry topic needed for arming/control.
- If VectorNav data goes stale, status reflects that clearly.

## Milestone 7: Keyboard Teleop

Create `tardigrade_teleop`.

Responsibilities:

- read keyboard commands
- publish robot-level velocity commands
- call arm/disarm/external-control services when explicitly requested
- never publish PX4 messages directly

Initial command shape:

- surge
- sway
- heave or depth intent
- yaw rate
- deadman/enable behavior

Acceptance criteria:

- Teleop can publish neutral commands.
- Teleop can publish simple directional commands.
- `tardigrade_px4` receives robot-level commands and translates them to PX4 Offboard setpoints.
- Commands time out to neutral when keyboard input stops.

## Milestone 8: Bringup Launch Files

Create `tardigrade_bringup`.

Initial launch modes:

- `mock.launch.py`: mock PX4 status plus PX4 driver
- `hardware.launch.py`: VectorNav, state estimation, PX4 driver, and teleop-ready robot APIs

Keep Micro XRCE-DDS Agent startup documented even if it is not launched by ROS at first.

Acceptance criteria:

- One command can start mock development mode.
- One command can start hardware bringup mode after the agent and devices are ready.
- Launch files do not hide safety-critical service calls.

## End-Of-Week Acceptance Criteria

The week-one milestone is successful when:

- Docker builds the required packages.
- VectorNav publishes usable ROS topics.
- Micro XRCE-DDS exposes PX4 topics.
- `tardigrade_px4` publishes Offboard heartbeat.
- Robot-level services can send arm/disarm/external-control commands.
- State estimation publishes odometry from VectorNav-derived data.
- PX4 receives odometry in the expected format.
- Keyboard teleop publishes robot-level commands.
- PX4 driver translates teleop commands into Offboard setpoints.
- Mock and hardware launch paths exist.

## Explicit Non-Goals For This Week

- Full custom EKF.
- Gate detection.
- BehaviorTree.CPP integration.
- Mission autonomy.
- Direct thruster testing as part of normal teleop.
- Recreating the legacy repo.

## Guidance For Working With Codex

You write the code. Use Codex for:

- package/interface review
- implementation sequencing
- debugging build errors
- checking ROS/PX4 assumptions
- code review before committing
- keeping these docs updated when decisions change
