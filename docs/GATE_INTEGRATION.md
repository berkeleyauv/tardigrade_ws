# Gate Mission Integration (RoboSub 2026 Task 1)

How the perception model (UR-B-Perception), the VectorNav IMU, and the Pixhawk
fit together to drive the robot through the gate.

## Architecture

```
ZED camera ──> gate_detector ──> /tardigrade/perception/gate ──┐
                (perception)        (GateDetection msg)        │
                                                               v
VectorNav ──> vectornav_odometry ──> /tardigrade/state/    gate_mission
  (IMU)                                 odometry          (state machine)
                  │                        │                   │
                  │                        │ (yaw for          │
                  │                        │  heading hold)    v
                  │                        │          /tardigrade/cmd_vel
                  │                        │           (Twist, body FLU)
                  v                        v                   │
        odometry_to_px4          pixhawk_interface <───────────┘
                  │              (body FLU -> world NED)
                  v                        │
   /fmu/in/vehicle_visual_odometry   /fmu/in/trajectory_setpoint
                  └──────────> PX4 <───────┘
```

New/changed pieces:

* `tardigrade_perception/gate_detector` — the ClassicalOrientation.py pipeline
  as a ROS node. Finds the gate ROI from the RED panels (LAB a-channel), then
  measures the black panels in the upper-left / lower-right quadrants exactly
  like the original script. Publishes `GateDetection`: `yaw_signal`
  (perspective cue from panel width ratio), `lateral` (normalized gate offset,
  -1..1), `gate_width_frac` (crude distance proxy).
* `tardigrade_interfaces/GateDetection.msg` — the new message.
* `tardigrade_mission/gate_mission` — state machine, see below.
* `tardigrade_px4/pixhawk_interface` — now subscribes `/tardigrade/cmd_vel`
  (body-frame FLU Twist), rotates it into world NED using the IMU yaw, and
  puts it into the TrajectorySetpoint it already publishes at 10 Hz. Commands
  older than `cmd_vel_timeout` (0.5 s) fail safe to zero velocity.

## Mission state machine

IDLE → SUBMERGE → SEARCH → ALIGN → THROUGH → DONE

* **SUBMERGE** — timed open-loop descent (`submerge_speed` for
  `submerge_duration`). Open loop because there is no depth sensor (see
  Limitations).
* **SEARCH** — yaw in place (`search_yaw_rate`) until the gate is detected for
  `detect_confirm_frames` consecutive frames.
* **ALIGN** — visual servoing at 10 Hz:
  * sway = `-kp_lateral * lateral` (centers the gate in frame)
  * yaw rate = `kp_yaw * yaw_signal` (squares up to the gate plane)
  * creep forward when roughly centered, full `approach_speed` when aligned
  * gate lost for `lost_timeout` → back to SEARCH
  * `gate_width_frac ≥ close_width_frac` → commit to THROUGH
* **THROUGH** — the gate is too close to see; hold the IMU heading captured at
  the transition and drive forward `through_speed` for `through_duration`
  (dead reckoning). Then DONE (zero velocity).

Safety: `/tardigrade/mission/abort` service stops everything; a
`mission_timeout` (180 s) hard-stops; stale cmd_vel → zero setpoint.

## The IMU's role

The VectorNav can't give you position or linear velocity (it would drift in
seconds from double-integration). What it *is* good for:

1. **Heading hold** in THROUGH, when the camera can't see the gate.
2. **Attitude reference for PX4** via `/fmu/in/vehicle_visual_odometry`.
3. Deciding search direction / measuring how far you've yawed.

So: camera answers "where is the gate relative to me", IMU answers "which way
am I pointing". The mission fuses them: vision servoing while the gate is
visible, IMU dead reckoning when it isn't.

## Build & run

```bash
# inside the container
cd /ws
colcon build --symlink-install
source install/setup.bash

# start the ZED wrapper and the PX4 DDS agent as usual, then:
ros2 launch tardigrade_bringup gate.launch.py port:=/dev/ttyUSB1

# operator sequence
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/mission/start std_srvs/srv/Trigger
```

Watch perception in rviz2 / rqt_image_view: `/tardigrade/perception/gate_debug`.

## Test plan (do these in order)

1. **Bench, no water:** `ros2 topic echo /tardigrade/perception/gate` while
   pointing the ZED at a printed/mock gate. Tune `a_threshold` (red) and
   `black_percentile` using pool footage. You can also test the pure pipeline
   offline: `python3 -c "import cv2; from tardigrade_perception import
   gate_pipeline as g; print(g.analyze_frame(cv2.imread('Gate1.png')))"`.
2. **Bench, dry, props off:** run the full launch, start the mission, verify
   `/tardigrade/cmd_vel` and `/fmu/in/trajectory_setpoint` look sane as you
   move the gate picture around. Verify the state transitions in the logs.
3. **Pool, tethered, slow gains:** default speeds are deliberately slow
   (0.2–0.4 m/s). Verify SUBMERGE depth (tune `submerge_duration`), then
   SEARCH/ALIGN against the real gate.
4. **Full runs**, tune `close_width_frac` and `through_duration` so the
   vehicle commits at the right distance and clears the gate.

## Limitations & gotchas (important)

* **PX4 offboard velocity mode needs a valid EKF velocity estimate.** Your
  visual odometry currently provides orientation only (velocity variance
  999). PX4's EKF2 may refuse to arm in offboard or may drift. Check
  `EKF2_EV_CTRL` / estimator status in QGroundControl on the bench *first*.
  If PX4 rejects offboard velocity control, the fallback is attitude+thrust
  control (a bigger change — ask if you need it).
* **No depth sensor = no closed-loop depth.** SUBMERGE is timed, and depth
  during ALIGN/THROUGH relies on the vehicle being trimmed near-neutral.
  A ~$100 Bar30/pressure sensor on the Pixhawk (or plumbed into PX4 as
  external baro) is the single highest-value hardware addition before
  competition — it turns depth into a solved problem.
* **The gate ROI finder replaces YOLO for now.** It keys on red panels; if
  pool lighting washes out red at distance, tune `a_threshold` down, or swap
  `find_gate_roi()` for a YOLO detector later — it's the only function that
  changes.
* **Style points / role choice are not attempted** — this goes through the
  gate center-ish. Once the core pass works, choosing a side = biasing
  `lateral` target left/right of the divider; a 90° yaw spin can be added as
  an extra state between ALIGN and THROUGH.
* Frame conventions: cmd_vel is **body FLU** (+x fwd, +y left, +z up,
  +yaw CCW). `pixhawk_interface` rotates to NED using the IMU yaw. If the
  vehicle strafes the wrong way in the pool, check the VectorNav mounting
  orientation before touching signs in code.
