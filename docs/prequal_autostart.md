# Automatic ESP Prequal Bringup

`prequal_autonomy.launch.py` starts the VectorNav driver, orientation-only
odometry, attitude controller, ESP thruster bridge, and ESP-native prequal
mission. It does not start or subscribe to the ZED.

The mission does not use the ignored legacy PX4 package. It publishes to
`/tardigrade/cmd_vel/manual`, and the depth/attitude controller publishes the
stabilized command consumed by the ESP bridge.

In IMU-only mode, descent and straight legs use calibrated command durations.
The 180-degree turn and heading during both straight legs remain closed-loop
using VectorNav orientation and angular velocity. An IMU cannot observe depth
or translation without unbounded integration drift, so the software cannot
verify 1.5 m or 20 m until a pressure/depth or position sensor is added.

## Safety behavior

- `dry_run` defaults to `true`.
- No nonzero command is sent until fresh odometry and healthy ESP status exist.
- After readiness, the mission publishes no command for `startup_delay_sec`,
  keeping the controller stale and the bridge at neutral.
- Stale odometry, stale ESP status, a serial error, or a phase timeout aborts.
- Excessive tilt aborts. Depth and cross-track checks are unavailable in
  IMU-only mode.
- The bridge returns to neutral if command input becomes stale.
- A physical kill switch remains the primary emergency stop.

The delay starts after all inputs become ready. Consequently movement occurs
no earlier than 60 seconds after the software stack is ready, and normally
later than one minute after robot power-on.

## Install on the Jetson

Build the hardware workspace before enabling automatic startup:

```bash
cd /home/auv/Developer/tardigrade_ws
./docker-build.sh --build
sudo env WORKSPACE="$PWD" ./docker/run_jetson_hardware.sh
```

Inside that temporary container:

```bash
cd /ws
./build.sh
exit
```

On the Jetson host, identify both serial devices:

```bash
ls -l /dev/serial/by-id/
```

Install and edit the configuration and service:

```bash
sudo mkdir -p /etc/tardigrade
sudo cp config/prequal.env.example /etc/tardigrade/prequal.env
sudo nano /etc/tardigrade/prequal.env
sudo cp docker/tardigrade-prequal.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tardigrade-prequal.service
```

Keep `DRY_RUN=true` for the first automatic test:

```bash
sudo systemctl start tardigrade-prequal.service
sudo journalctl -fu tardigrade-prequal.service
```

Confirm that the log reports fresh odometry, healthy ESP status, and a passed
dry run. Then stop the service, set `DRY_RUN=false`, and only start it in a
movement-safe test environment:

```bash
sudo systemctl stop tardigrade-prequal.service
sudo nano /etc/tardigrade/prequal.env
sudo systemctl start tardigrade-prequal.service
sudo journalctl -fu tardigrade-prequal.service
```

After validation, a normal Jetson boot starts the container and launch file.
Disable automatic startup with:

```bash
sudo systemctl disable --now tardigrade-prequal.service
```
