# Thruster Mapping

The mixer source of truth is:

```text
src/tardigrade_esp/config/esp_thruster_map.json
```

The ESP firmware pin order must match the same slot order. ROS service and
documentation slot numbers are 1-based; the serial protocol encodes motor
indices 0–7.

## Robot Frame

All mixer coefficients use ROS FLU body coordinates:

```text
+X forward
+Y left / port
+Z up
positive roll  right-hand rotation about +X
positive pitch right-hand rotation about +Y
positive yaw   nose left / counter-clockwise from above
```

## Current Physical And Electrical Map

The current sticky-note wiring map is:

| Slot | ESP pin | Physical location | Role |
|---:|---:|---|---|
| 1 | 21 | front left | vectored |
| 2 | 19 | rear right | vectored |
| 3 | 27 | front left | vertical |
| 4 | 18 | rear left | vertical |
| 5 | 5 | rear right | vertical |
| 6 | 14 | front right | vertical |
| 7 | 12 | front right | vectored |
| 8 | 26 | rear left | vectored |

The current nonzero mixer coefficients are:

| Slot | Surge | Sway | Heave | Roll | Pitch | Yaw |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | +1 | −1 | 0 | 0 | 0 | −1 |
| 2 | +1 | −1 | 0 | 0 | 0 | +1 |
| 3 | 0 | 0 | +1 | +1 | −1 | 0 |
| 4 | 0 | 0 | +1 | +1 | +1 | 0 |
| 5 | 0 | 0 | +1 | −1 | +1 | 0 |
| 6 | 0 | 0 | +1 | −1 | −1 | 0 |
| 7 | +1 | +1 | 0 | 0 | 0 | +1 |
| 8 | +1 | +1 | 0 | 0 | 0 | −1 |

These coefficients assume a positive motor command produces the intended
positive force direction. Propeller, ESC, or wiring polarity can invalidate
that assumption even when the slot number is correct.

## Current Command Path

```text
Xbox / Jetson controller
  -> /tardigrade/cmd_vel
  -> thruster_mixer
  -> /tardigrade/thrusters/cmd (8 normalized values)
  -> esp_bridge
  -> binary SetMotor packets
  -> ESP safety clamp and PWM
```

The ESP does not receive the mix matrix and does not infer robot motion. It
only knows motor index and normalized command. The Jetson owns the mapping.

## Verify Slot Identity

Make the vehicle physically safe, clear every propeller, assign a kill-switch
operator, and stop every other command mode and ESP bridge.

Start the bounded checkout mode:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Arm:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Test one slot for one second:

```bash
ros2 service call /tardigrade/test/run_thruster \
  tardigrade_interfaces/srv/TestThruster \
  "{slot: 1, command: 0.10, duration_sec: 1.0}"
```

For each slot record:

- which physical thruster moved;
- whether a positive command produces the assumed force direction;
- whether the motor remained neutral before and after the request;
- any slot that did not move at the bounded checkout authority.

The checkout node commands the other seven slots to zero and returns all eight
to zero automatically. Do not test multiple slots simultaneously while
identifying wiring.

Disarm after each mapping session:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

## Verify Body-Axis Signs

After slot identity and polarity are known, use direct mode with thruster power
disconnected first:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py
```

Watch the final eight values:

```bash
ros2 topic echo /tardigrade/thrusters/cmd
```

Hold LB and command one axis at a time. Compare the signs against the table.
Then conduct only brief, restrained powered taps using the end-to-end pool
runbook.

If the wrong thruster moves, fix slot/pin identity. If the correct thruster
pushes in the opposite direction, reverse its physical/ESC polarity or invert
all nonzero mixer coefficients for that slot. Do not hide a hardware or mixer
error by reversing pilot controls.

Repeat this verification after any PDB rewiring, ESC replacement, propeller
change, firmware pin-order change, or mixer edit.
