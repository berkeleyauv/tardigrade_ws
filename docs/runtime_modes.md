# Tardigrade Runtime and Checkout Modes

Run exactly one actuator backend at a time. Sensor and Foxglove processes may
run alongside a mode, but `thruster_checkout_real` and `pool_assisted` each own
the ESP serial port and must never overlap.

## Monitoring

Start rosbridge and connect Foxglove to `ws://JETSON_IP:9090`:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Use `pool_checkout.json` for bounded thruster checkout, `pool_operator.json`
or `pool_sensors.json` for normal hardware work, and `sim_operator.json` or
`sim_sensors.json` for Unity. The shared `pid_tuning.json` works with either
backend. Install Tardigrade Tools first as described in `foxglove/README.md`.

## Hardware sensors

The diagnostic sensor launches are intentionally retained:

```bash
ros2 launch tardigrade_bringup zed_state.launch.py
ros2 launch tardigrade_bringup vectornav_state.launch.py
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Assisted control consumes `/tardigrade/state/odometry/filtered`. Verify the
sensor rates, REP-103 signs, frames, and TF tree before enabling thrust.

## Individual hardware thruster checkout

Stop every teleop, controller, allocator, and other ESP bridge. Start only:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Confirm ESP telemetry, arm deliberately, then request one physical slot:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"

ros2 service call /tardigrade/test/run_thruster \
  tardigrade_interfaces/srv/TestThruster \
  "{slot: 1, command: 0.05, duration_sec: 1.0}"
```

Slots are 1-indexed. The checkout rejects invalid or non-finite requests,
commands above 0.10, durations above two seconds, and a second request while a
test is active. Any rejection or timeout publishes a named eight-thruster
neutral command. Disarm after every observation:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

## Assisted Xbox control

After the hardware sensor and individual-thruster gates pass:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

The launch uses the complete production path:

```text
/joy -> stamped manual velocity -> source mux -> velocity controller
  -> wrench allocator -> force/command mapping -> named command -> ESP bridge
```

LB is the continuous deadman. Stale Joy, velocity, odometry, allocation, or
actuator input neutralizes the chain. Use `start_joy_node:=true` only when the
controller is connected directly to the Jetson; otherwise Foxglove publishes
`/joy` from the operator computer.

## Unity

```bash
ros2 launch tardigrade_bringup unity_sil.launch.py
ros2 launch tardigrade_bringup unity_operator.launch.py
```

`unity_sil` starts ROS-TCP, the shared controller, estimator, and TF.
`unity_operator` additionally starts rosbridge and selects the manual command
source by default. Neither launch starts ESP hardware.

## Qualification missions

The preserved `prequal_autonomy.launch.py` and `qual_autonomy.launch.py`
profiles are hardware modes. They start the production controller and ESP
bridge and default to `dry_run:=true`. Follow the pool runbook and inspect every
argument before changing that value.

## Recording

Record every powered attempt:

```bash
ros2 bag record -o pool_checkout_01 \
  /joy /zed/zed_node/odom /vectornav/imu \
  /tardigrade/sensors/imu \
  /tardigrade/state/odometry/filtered \
  /tardigrade/control/velocity_setpoint/manual \
  /tardigrade/control/velocity_setpoint \
  /tardigrade/control/wrench_command \
  /tardigrade/control/allocation_status \
  /tardigrade/actuators/thruster_forces \
  /tardigrade/actuators/thruster_commands \
  /tardigrade/esp/state /tf /tf_static
```
