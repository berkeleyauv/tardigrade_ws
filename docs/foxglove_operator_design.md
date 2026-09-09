# Foxglove operator design for Tardigrade

## Executive summary

Tardigrade now uses one visual language for Unity and the physical vehicle.
The operator should not need to relearn where safety state, orientation,
cameras, or controller information lives when moving from a laptop simulation
to the pool deck. Backend-specific differences are limited to camera topic
names, Unity reset, and the deliberately isolated real-thruster checkout.

Three Berkeley AUV Foxglove panels provide the pieces that built-in panels did
not express cleanly:

1. **Vehicle Status**: safety gates, topic freshness, allocation state, and a
   physical eight-thruster display.
2. **Attitude**: artificial horizon, ENU compass, depth, and body motion from
   filtered odometry.
3. **PID Tuner**: one selected axis, one response chart, effort/saturation,
   P/I/D contributions, and atomic gain updates.

This removes the primary failure visible in the previous PID layout: many tiny
plots whose floating legends occupied most of each panel. It also avoids
plotting values with different physical dimensions on the same y-axis.

## Research findings

Foxglove layouts are JSON-backed workspace arrangements that can be imported,
exported, and shared. That makes committed layouts a suitable, reviewable part
of the robot configuration rather than an operator-specific browser state.[^1]
The extension API gives a custom panel message subscriptions, render updates,
persisted panel state, publishing, and ROS service calls. Tardigrade uses only
subscriptions, local state, and service calls; the status and attitude panels
do not publish control data.[^2]

Built-in panels remain the right choice where their semantics already match
the job. The Image panel handles ROS camera images, while the 3D panel can
render transforms and pose/odometry data in a fixed frame.[^3][^4] Therefore
camera and spatial visualization stay built-in. Custom code is reserved for
robot-specific interactions and compact instruments.

Duke Robotics separates PID editing, control toggles, system/sensor status,
and thruster allocation into distinct Foxglove extensions.[^5] Its PID panel
uses a table-like edit/submit interaction rather than forcing operators to
write service JSON by hand.[^6] Those are strong interaction patterns, but
Duke's current code targets its own messages and ROS 2 stack. The Berkeley
extension applies the ideas to Tardigrade's Foxy-compatible interfaces with an
original Apache-2.0 implementation. No Duke source was copied.

Underwater vehicle interfaces also benefit from familiar instruments. Blue
Robotics Cockpit documents a virtual horizon, compass, depth display, and
compact telemetry widgets as operator-facing components.[^7] Tardigrade's
Attitude panel adopts that instrument vocabulary while retaining REP-103
semantics: filtered ENU/FLU odometry is the source, yaw zero is +X/East, and
depth is displayed as `-z`.

## Information architecture

The three canonical workflows answer different questions:

| Layout | Operator question | Dominant representation |
|---|---|---|
| Operator | “Is it safe, and where is the vehicle going?” | status, 3D, horizon, stereo cameras |
| PID tuning | “Does one loop track well, and why is its output shaped this way?” | one response trace, effort meter, terms, gains |
| Sensors/EKF | “Which measurement or estimate is wrong?” | stereo cameras, 3D, horizon, two focused comparisons |
| Pool checkout | “Did exactly one named motor run and neutralize?” | status, bounded service, thruster bars |

The simulation and pool variants share panel placement. The operator layout
always puts Vehicle Status on the left, spatial and attitude context at the
upper right, and cameras below. Sensor layouts always put stereo images on top
and estimation/status diagnostics below. The exact same PID layout is used in
both environments because the controller and its ROS contract are shared.

## Why the PID view is different

The prior layout showed all six axes simultaneously and repeated three traces
per chart. On a normal laptop screen, labels became the most prominent visual
element and the actual response was compressed. The new view asks the operator
to choose the axis being tuned and then gives that loop enough space.

Only setpoint and measurement appear on the response chart. They have the same
units and direct comparison is meaningful. Output has different units—N for a
linear axis and N·m for an angular axis—so it appears as a normalized signed
meter with a saturation state. P, I, and D contributions are numeric cards;
their immediate purpose is causal diagnosis, not long-term trajectory review.

Gain edits are made as one service request containing `kp`, `ki`, `kd`,
`integral_limit`, and `output_limit`. This avoids partially applied gain sets.
The controller publishes those live values in `PidDebug`, including the newly
added integral limit, so “Reload live values” reflects the running controller
rather than stale form state. Applied gains remain temporary until copied into
the canonical controller YAML.

## Safety model

The Vehicle Status panel treats armed as a hazardous state and colors it red.
Disarm is immediate; arm requires two clicks within five seconds. External
control buttons only appear when a fresh Unity status message identifies the
backend as simulation. The panel never publishes `ThrusterCommands`.

Real individual-thruster testing remains in `pool_checkout.json` through
`/tardigrade/test/run_thruster`. The protected ROS service keeps the one-slot,
amplitude, duration, neutralization, and watchdog constraints. Combining the
bounded service with live named bars makes the important assertion visible:
exactly one intended physical slot becomes nonzero, then all eight return to
zero.

## Plot policy

Plots are kept when temporal shape is the question: tracking response, depth
estimate comparison, or pressure history. Canonical built-in plots have no
more than three traces, use short labels, and place the legend at the top rather
than floating over data. Current-value state, boolean gates, orientation, and
eight simultaneous actuator values use purpose-built instruments instead.

This policy is enforced by `foxglove/validate_layouts.py`. Layout JSON is
generated by `foxglove/generate_layouts.py`, which also ensures the shared
simulation/pool structure does not drift through manual editing.

## Remaining fidelity and UX work

The current 3D view is useful for frame motion and odometry but the URDF still
has limited visual geometry. Adding a lightweight visual robot mesh to
`tardigrade_description` would make roll/pitch/yaw easier to interpret without
changing physics. The 3D panel can render URDF and transform-based layers, so
that improvement belongs in description/TF configuration rather than another
custom panel.[^3]

After the first pool session, review recorded data before adding widgets.
Candidate additions should solve observed operator mistakes—for example, a
camera-staleness overlay or estimator innovation summary—not simply expose
every available field. Retain raw-message layouts for engineering diagnosis,
but keep them outside the normal operator flow.

## Sources

[^1]: Foxglove, “Layouts,” [https://docs.foxglove.dev/docs/visualization/layouts](https://docs.foxglove.dev/docs/visualization/layouts).
[^2]: Foxglove, “Create a custom panel” and `PanelExtensionContext`, [https://docs.foxglove.dev/docs/extensions/guides/create-custom-panel](https://docs.foxglove.dev/docs/extensions/guides/create-custom-panel), [https://docs.foxglove.dev/docs/extensions/extension-api/type-aliases/PanelExtensionContext](https://docs.foxglove.dev/docs/extensions/extension-api/type-aliases/PanelExtensionContext).
[^3]: Foxglove, “3D panel,” [https://docs.foxglove.dev/docs/visualization/panels/3d](https://docs.foxglove.dev/docs/visualization/panels/3d).
[^4]: Foxglove, “Image panel,” [https://docs.foxglove.dev/docs/visualization/panels/image](https://docs.foxglove.dev/docs/visualization/panels/image).
[^5]: Duke Robotics, `robosub-ros2/foxglove`, [https://github.com/DukeRobotics/robosub-ros2/tree/main/foxglove](https://github.com/DukeRobotics/robosub-ros2/tree/main/foxglove).
[^6]: Duke Robotics, “PID Panel,” [https://github.com/DukeRobotics/robosub-ros2/blob/main/foxglove/extensions/pid-panel/README.md](https://github.com/DukeRobotics/robosub-ros2/blob/main/foxglove/extensions/pid-panel/README.md).
[^7]: Blue Robotics, “Cockpit advanced usage,” [https://blueos.cloud/cockpit/docs/latest/usage/advanced/](https://blueos.cloud/cockpit/docs/latest/usage/advanced/).
