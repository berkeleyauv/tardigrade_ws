# Unity–ROS 2 Software-in-the-Loop Simulator

The Unity backend is a physical plant. Production control and mission nodes do
not subscribe to simulator truth. The load-bearing path is:

```text
mission/teleop -> TwistStamped velocity setpoint -> velocity controller
  -> WrenchStamped -> bounded geometry allocator -> named forces
  -> thrust-curve mapper -> named normalized commands -> Unity Rigidbody
  -> IMU + pressure + synthetic VIO + stereo images -> EKF/ROS consumers
```

The Unity project is expected next to this repository as
`../tardigrade_unity_world`. Both repositories use the branch
`feature/realistic-unity-ros2-sim`.

## Build and start

For interactive editor, Foxglove, and keyboard testing, follow the focused
[Unity operator workflow](unity_operator_workflow.md). It excludes missions and
is the normal control-development entry point.

Build ROS inside the Foxy container:

```bash
./docker-build.sh --build
./build.sh
source install/setup.bash
ros2 launch tardigrade_bringup unity_sil.launch.py
```

To drive Unity with short keyboard pulses instead of a mission, select the
manual source and run the keyboard node in a second interactive terminal:

```bash
ros2 launch tardigrade_bringup unity_sil.launch.py active_source:=manual
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Each motion key publishes both a stamped manual velocity and a matching
short-lived enable signal. When the pulse expires, both return to neutral.

Open `tardigrade_unity_world` in Unity 6000.2.6f2, configure the ROS connection
for ROS 2 and TCP port 10000, open `Assets/Scenes/SampleScene.unity`, and press
Play. The existing `RosRobotBridge` scene component creates the rigid body,
collision hull, plant modules, gate, sensors, and cameras at runtime.

For a standalone player, set `UNITY_EDITOR` and build one target:

```bash
cd ../tardigrade_unity_world
./scripts/validate-simulation.sh
./scripts/validate-playmode.sh
./scripts/build-standalone.sh linux
```

On Windows use `scripts/build-standalone.ps1 -Target windows`. Once built,
`scripts/run-unity-sil.sh` or `scripts/run-unity-sil.ps1` starts the ROS Docker
stack and player together. Override the player with `SIM_PLAYER`.

## Interfaces

| Topic/service | Type | Owner/consumer |
|---|---|---|
| `/tardigrade/control/velocity_setpoint/{manual,mission,pose}` | `geometry_msgs/TwistStamped` | isolated command sources -> mux |
| `/tardigrade/control/pose_setpoint` | `geometry_msgs/PoseStamped` | optional `odom`-frame pose target -> guidance |
| `/tardigrade/control/velocity_setpoint` | `geometry_msgs/TwistStamped` | selected fresh source -> controller; `base_link` |
| `/tardigrade/control/velocity_setpoint_enabled` | `std_msgs/Bool` | mux authorization/freshness -> controller |
| `/tardigrade/control/pose_setpoint_enabled` | `std_msgs/Bool` | pose guidance validity -> mux |
| `/tardigrade/control/wrench_command` | `geometry_msgs/WrenchStamped` | controller -> allocator; N and N m |
| `/tardigrade/control/allocation_status` | `tardigrade_interfaces/AllocationStatus` | achieved wrench, residual, feasibility, saturation |
| `/tardigrade/control/{axis}/debug` | `tardigrade_interfaces/PidDebug` | six live velocity-loop debug streams for tuning |
| `/tardigrade/actuators/thruster_forces` | `tardigrade_interfaces/ThrusterForces` | allocator -> actuator map; named N |
| `/tardigrade/actuators/thruster_commands` | `tardigrade_interfaces/ThrusterCommands` | actuator map -> exactly one backend; named `[-1,1]` |
| `/tardigrade/sensors/imu/data` | `sensor_msgs/Imu` | Unity -> EKF; `imu_link` |
| `/tardigrade/sensors/pressure` | `sensor_msgs/FluidPressure` | Unity -> depth conversion; absolute Pa |
| `/tardigrade/sensors/visual_odometry` | `nav_msgs/Odometry` | Unity -> EKF; noisy/drifting, `odom` |
| `/tardigrade/sensors/camera/front/{left,right}/{image_raw,camera_info}` | standard sensor messages | Unity -> ROS image consumers |
| `/tardigrade/state/odometry/filtered` | `nav_msgs/Odometry` | EKF -> sole controller state input |
| `/tardigrade/sim/ground_truth/odometry` | `nav_msgs/Odometry` | tests/Foxglove only |
| `/clock` | `rosgraph_msgs/Clock` | Unity; monotonic 100 Hz simulation time |
| `/tardigrade/sim/reset` | `tardigrade_interfaces/ResetSimulation` | scenario ID, seed, ENU initial pose |
| `/tardigrade/control/set_velocity_pid_gains` | `tardigrade_interfaces/SetVelocityPidGains` | validated live physical-unit gains; resets PID state |
| `/tardigrade/control/reset_pid` | `std_srvs/Trigger` | clear velocity-loop state without changing gains |

Named actuator arrays are rejected when lengths differ or a value/name is
missing, duplicated, unknown, non-finite, or outside its range. Unity requires
arming and external-control services and returns every thruster toward neutral
when commands are older than 0.5 seconds.

## Frames and time

All public ROS values use SI, FLU body axes, ENU world axes, and right-handed
angular pseudovectors. Camera messages use optical frames. The frame tree is:

```text
map       (global correction owner; not published by Unity)
└── odom  (continuous EKF world frame)
    └── base_link  (published by robot_localization)
        ├── imu_link
        ├── pressure_link
        └── zed_camera_link
            ├── zed_left_camera_optical_frame
            └── zed_right_camera_optical_frame
```

`robot_state_publisher` owns fixed sensor transforms. Unity ground truth is a
message, never a competing TF publisher. Sensor headers carry acquisition time;
latency/jitter queues only delay delivery. Reset clears actuator and sensor
histories but does not rewind `/clock`.

## Vehicle configuration

Edit only
`src/tardigrade_description/config/vehicle.json`. It contains mass/inertia,
CoM/CoB/displacement, water and damping, buoyancy elements, exact actuator
geometry/curves, sensor extrinsics/rates/noise, uncertainty, and provenance.
Initial numerical values are explicitly marked estimates pending measurement.

Generate or check consumer files from the ROS environment:

```bash
ros2 run tardigrade_description export_vehicle_config \
  --source src/tardigrade_description/config/vehicle.json \
  --unity-output ../tardigrade_unity_world/Assets/StreamingAssets/tardigrade_vehicle.json \
  --allocator-output src/tardigrade_description/config/allocator.json

ros2 run tardigrade_description export_vehicle_config --check \
  --source src/tardigrade_description/config/vehicle.json \
  --unity-output ../tardigrade_unity_world/Assets/StreamingAssets/tardigrade_vehicle.json \
  --allocator-output src/tardigrade_description/config/allocator.json
```

Both generated files record the source SHA-256. ROS CI checks the allocator;
the local cross-repository test checks Unity too.

The physics model uses one collider-bearing `Rigidbody`, physical weight,
distributed partially submerged Archimedes forces, current-relative body-axis
linear/quadratic damping, and thruster `AddForceAtPosition`. PhysX cannot
represent a full directional translational mass tensor, so the initial Unity
projection uses mean diagonal added mass for translation and diagonal effective
inertia for rotation. It never differentiates acceleration into a feedback
force. Record this approximation when fitting pool data.

## Deterministic scenarios

Call reset before a run, for example:

```bash
ros2 service call /tardigrade/sim/reset \
  tardigrade_interfaces/srv/ResetSimulation \
  "{scenario_id: gate_nominal, seed: 42, initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.173648, w: 0.984808}}}"
```

Supported profiles are `clean`, `gate_nominal`, `gate_cross_current`,
`current_impulse`, `low_visibility`, `sensor_stress`, `sensor_dropout`,
`delayed_messages`, `vio_reset`, `failed_thruster_1` through
`failed_thruster_8`, and `derated_thruster_1`. Noise and faults are reproducible
from the recorded seed, scenario, and config hash. Determinism is tolerance-
based across operating systems rather than bit-identical PhysX state.

## Acceptance and telemetry

Use Unity for physics, sensor, estimator, controller, and actuator-failure
tests. Record `/clock`, raw sensors, filtered odometry, ground truth, wrench,
named forces/commands, `/tf`, and `/tf_static`. Foxglove should display truth
versus estimate, camera images, controller output, allocation residual, and
each named thruster. Production controllers must never subscribe to simulator
ground truth.

## Visual and camera fidelity

The runtime-generated pool visual shell follows the same dimensions and pose as
the independent collision shell. It includes tiled surfaces, scale-bearing lane
marks, end-wall T marks, a wet deck, a deterministic animated water surface,
subsurface lighting, haze, and suspended particulate. The gate art and collision
geometry also remain separate so art changes cannot silently change a test.

Scenario reset selects the visual profile as well as the physical/sensor fault
profile:

- `clean`: no current or stochastic sensor faults, clear water, minimal camera
  grain; use this to diagnose physics and transforms.
- ordinary scenarios: representative pool haze, color attenuation, particulate,
  lens falloff, and camera grain.
- `low_visibility`: stronger scattering/color loss and particulate for perception
  stress testing.
- `sensor_stress`: nominal water with stronger synthetic camera sensor noise.

The two ROS camera publishers use short latest-frame queues and pause rendering
while the ROS-TCP endpoint reports a connection failure. This prevents raw stereo
frames from filling the transport queue while ROS is unavailable. Visual wave,
light, and camera-noise phase restarts from the reset seed even though `/clock`
correctly remains monotonic.

See `../tardigrade_unity_world/ASSET_PROVENANCE.md` before importing external
models or materials. Imported HDRP and built-in-pipeline materials must be
converted to URP and checked for unsupported shaders before they are committed.

## Characterization workflow

Store raw bags and reports outside Git/LFS according to team data policy, and
commit a manifest/report under `characterization/`. Each fit records date,
vehicle revision, battery voltage, water conditions, source bag URI/hash,
method, coefficient with units and uncertainty, residual plots, reviewer, and
the resulting vehicle-config hash. Update parameter values without adding
one-off forces or sensor exceptions to model code.

Until measurements replace the estimates, simulation results are suitable for
software integration and sign/safety testing—not claims of pool-level fidelity.
