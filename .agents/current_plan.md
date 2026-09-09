# Current Development Plan

## Active goal

Prepare the current ROS and Unity control path for simulator development and
near-term pool testing without changing the working ESP firmware contract.

Priorities:

1. Validate Unity sensors, filtered state, feedback, allocation, and named
   actuator output.
2. Preserve and test ZED, VectorNav, ESP telemetry, arming, watchdog, and the
   bounded individual-thruster checkout.
3. Use Foxglove layouts for simulator operation, sensor diagnosis, pool
   checkout, and physical-unit PID tuning.
4. Fit vehicle and sensor parameters from repeatable bagged pool experiments.
5. Keep hardware launches conservative until pool evidence proves a profile is
   redundant.

## Guardrails

- Do not edit firmware packet definitions or physical pin mapping as part of
  ROS cleanup.
- Do not run more than one ESP bridge or command-producing hardware mode.
- Do not auto-arm in launch files.
- Do not feed Unity ground truth into production control or estimation.
- Do not use the synthetic-pose bench hook during normal control.
- Do not remove ZED/VectorNav diagnostic profiles before their replacement is
  verified on the robot.
- Keep real-thrust work behind the checks in `docs/pool_teleop.md`.

## Canonical entry points

```bash
ros2 launch tardigrade_bringup unity_operator.launch.py
ros2 launch tardigrade_esp thruster_checkout_real.launch.py
ros2 launch tardigrade_bringup pool_assisted.launch.py
ros2 launch tardigrade_bringup prequal_autonomy.launch.py
ros2 launch tardigrade_bringup qual_autonomy.launch.py
```
