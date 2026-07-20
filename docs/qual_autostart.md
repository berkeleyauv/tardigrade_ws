# Automatic ESP Qualification Bringup

`qual_autonomy.launch.py` starts the ZED wrapper, VectorNav driver, EKF
odometry, depth/attitude controller, ESP thruster bridge, and the ESP-native
qual mission.

The mission publishes to
`/tardigrade/cmd_vel/manual`, and the depth/attitude controller publishes the
stabilized command consumed by the ESP bridge.

The active course is 1.5 m down, 20 m outbound on the captured starting
heading, a closed-loop 180-degree turn, and a position-seeking return to the
saved start. The mission and depth controller consume
`/tardigrade/state/odometry/filtered`. Forward effort remains fixed, while a
VectorNav-derived heading PD controller corrects yaw. Outbound completion uses
along-track distance, and the return continually points toward the saved start
instead of blindly running for a fixed time.

## Safety behavior

- `dry_run` defaults to `true`.
- No nonzero command is sent until fresh odometry and healthy ESP status exist.
- After readiness, the mission publishes no command for `startup_delay_sec`,
  keeping the controller stale and the bridge at neutral.
- Stale odometry, stale ESP status, a serial error, or a phase timeout aborts.
- Excessive tilt, depth error, or outbound cross-track error aborts.
- The bridge returns to neutral if command input becomes stale.
- A physical kill switch remains the primary emergency stop.

The delay starts after all inputs become ready. Consequently movement occurs
no earlier than 15 seconds after the software stack is ready, and normally
later than 15 seconds after robot power-on.

## Install on the Jetson

Build the hardware workspace before enabling automatic startup:

```bash
cd /home/auv/Developer/tardigrade_ws
./docker-build.sh --build
sudo env WORKSPACE="$PWD" ./docker-build.sh --jetson
```

Inside that temporary container:

```bash
cd /ws
./build.sh --hardware
exit
```

On the Jetson host, identify both serial devices:

```bash
ls -l /dev/serial/by-id/
```

Install and edit the configuration and service:

```bash
sudo mkdir -p /etc/tardigrade
sudo cp robot/autostart/qual.env.example /etc/tardigrade/qual.env
sudo nano /etc/tardigrade/qual.env
sudo cp robot/autostart/tardigrade-qual.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tardigrade-qual.service
```

Keep `DRY_RUN=true` for the first automatic test:

```bash
sudo systemctl start tardigrade-qual.service
sudo journalctl -fu tardigrade-qual.service
```

Confirm that the log reports fresh odometry, healthy ESP status, and a passed
dry run. Then stop the service, set `DRY_RUN=false`, and only start it in a
movement-safe test environment:

```bash
sudo systemctl stop tardigrade-qual.service
sudo nano /etc/tardigrade/qual.env
sudo systemctl start tardigrade-qual.service
sudo journalctl -fu tardigrade-qual.service
```

After validation, a normal Jetson boot starts the container and launch file.
Disable automatic startup with:

```bash
sudo systemctl disable --now tardigrade-qual.service
```
