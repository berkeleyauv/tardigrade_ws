# Foxglove operator UI

Foxglove is the common operator, state-estimation, sensor, and tuning UI for
both Unity and the physical vehicle. The canonical layouts deliberately keep
the same controls and instruments in the same places; only backend-specific
camera topics and reset/checkout controls differ.

## Install Tardigrade Tools once

The layouts use three original Berkeley AUV panels: **Vehicle Status**,
**Attitude**, and **PID Tuner**. Build the extension:

```bash
cd foxglove/extensions/tardigrade-tools
pnpm install --frozen-lockfile
pnpm run typecheck
pnpm run package
```

In Foxglove Desktop, open **Settings → Extensions → Install local extension**
and select:

```text
foxglove/extensions/tardigrade-tools/berkeleyauv.tardigrade-tools-0.1.0.foxe
```

Reload Foxglove after installing a new version. Foxglove Desktop and a
Foxglove developer seat are required for local extensions. For active panel
development, `pnpm run local-install` installs directly into Foxglove's local
extension directory.

## Connect

The ROS 2 Foxy workflow exposes Rosbridge on port 9090:

```bash
ros2 launch tardigrade_bringup unity_operator.launch.py
```

In Foxglove, add a **Rosbridge** connection to:

```text
ws://localhost:9090
```

For pool work, replace `localhost` with the Jetson's address. Do not select
**Foxglove WebSocket** for port 9090; it is a different protocol.

Import a layout with **Layouts → Add → Import from file**.

## Canonical layouts

| Workflow | Simulation | Pool / robot |
|---|---|---|
| Drive and observe | `layouts/sim_operator.json` | `layouts/pool_operator.json` |
| Tune control | `layouts/pid_tuning.json` | `layouts/pid_tuning.json` |
| Inspect sensors/EKF | `layouts/sim_sensors.json` | `layouts/pool_sensors.json` |
| Bounded motor checkout | not applicable | `layouts/pool_checkout.json` |

`sim_pid_tuning.json` is retained as an exact alias for existing Foxglove
imports. Generate and verify the canonical layout set with:

```bash
python3 foxglove/generate_layouts.py
python3 foxglove/validate_layouts.py
```

The older `zed.json`, `state_estimation.json`, `fused_state.json`, and ESP
telemetry layouts remain available as focused hardware diagnostics. They are
not the normal operator view.

## What the custom panels show

- **Vehicle Status** auto-detects Unity or the ESP backend. It combines
  arming, external-control state, controller gates, sensor freshness,
  allocation health, and eight named signed thruster bars. It has an immediate
  disarm button and a two-click arm confirmation. It never publishes a motor
  command.
- **Attitude** shows the filtered estimate as an artificial horizon and ENU
  compass tape, with depth and body velocities beside it. This is faster to
  scan than quaternion fields or three Euler-angle plots.
- **PID Tuner** selects one of six axes. The chart contains only setpoint and
  measurement because they share units. Controller effort, saturation, and
  P/I/D contributions are separate meters/readouts. Apply updates the selected
  axis atomically through
  `/tardigrade/control/set_velocity_pid_gains`; reload reads the live debug
  message; reset clears PID history without changing gains.

Plots are reserved for comparisons over time. Canonical built-in plots contain
at most three clearly labelled traces and never use a floating legend.

## Simulator sequence

1. Start Unity in Play mode and start `unity_operator.launch.py`.
2. Connect Foxglove to `ws://localhost:9090` and open `sim_operator.json`.
3. Reset the simulator. Reset disarms the plant and turns external control off.
4. In Vehicle Status, enable external control and use the two-click arm action.
5. Start teleop or mission commands. Confirm `controller`, `command`, `odom`,
   and `allocation` all remain healthy.
6. Use **Disarm now** before ending the run or changing configuration.

The default reset in the simulator operator layout preserves the established
`z=0 m` starting pose. The status panel intentionally displays **ARMED** in
red: armed is a hazardous state, not a success condition.

## PID tuning workflow

1. Use the deterministic `clean` scenario and record the seed.
2. Select one axis in PID Tuner and excite only that axis with a bounded step.
3. Tune response while watching setpoint versus measurement, output percentage,
   saturation, and the P/I/D contribution cards.
4. Confirm the result under a second seed and the intended sensor profile.
5. Copy accepted values into
   `src/tardigrade_control/config/control.yaml`; service changes are temporary.

Linear-axis setpoint/measurement units are m/s and output is N. Angular-axis
setpoint/measurement units are rad/s and output is N·m. Mixing these quantities
on one plot is intentionally avoided.

## Physical thruster checkout

Follow `docs/pool_teleop.md` and the team's physical safety procedure. Secure
the vehicle, remove propellers for dry testing where appropriate, and ensure no
other actuator publisher is active. Then:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py
```

Open `pool_checkout.json`, confirm the ESP link and mapping, arm explicitly,
then call `/tardigrade/test/run_thruster`. Verify the named command returns to
eight zeroes and disarm. The service is limited to one physical slot, a 0.10
normalized command, and two seconds; the ESP watchdog and firmware limits stay
authoritative.

## Source and design notes

The interaction model was informed by Duke Robotics' separate PID,
control-toggle, sensor-status, and thruster-allocation panels and by the compact
underwater instruments in Blue Robotics Cockpit. The Tardigrade extension is
an original Apache-2.0 implementation using Berkeley ROS interfaces; no Duke
source was copied. See `docs/foxglove_operator_design.md` for the cited design
study and future recommendations.
