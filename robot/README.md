# Robot Host Setup

This folder contains setup files that live with the repo but are installed or
used on the robot host, not inside the ROS workspace itself.

## Udev Rules

Linux assigns USB serial devices names such as `/dev/ttyUSB0` and
`/dev/ttyACM0`. Those names can change when devices are unplugged, rebooted, or
detected in a different order. That is risky for robot bringup because the ESP
and VectorNav can swap apparent port numbers.

Udev rules create stable symlinks based on device identity. The goal is to use
paths like:

```text
/dev/tardigrade_esp
/dev/tardigrade_vectornav
```

instead of guessing:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

An example rules file lives at:

```text
robot/udev/99-tardigrade.rules.example
```

Do not install it unchanged. Fill in the real device attributes from the
Jetson first.

## Find Device Attributes

Plug in one device at a time and check the stable by-id links:

```bash
ls -l /dev/serial/by-id/
```

For a candidate port, inspect udev attributes:

```bash
udevadm info -a -n /dev/ttyUSB0
```

Useful fields are usually `idVendor`, `idProduct`, and `serial`. Prefer rules
that include a serial number when available.

## Install On The Jetson

After editing the rules file for the real hardware:

```bash
sudo cp robot/udev/99-tardigrade.rules.example /etc/udev/rules.d/99-tardigrade.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/tardigrade_*
```

If the symlinks do not appear, unplug and replug the devices or reboot the
Jetson.

Once the symlinks are verified, use them in `/etc/tardigrade/*.env`:

```text
ESP_PORT=/dev/tardigrade_esp
VECTORNAV_PORT=/dev/tardigrade_vectornav
```

## Autostart

Automatic prequalification and qualification startup files live in:

```text
robot/autostart/
```

That folder contains systemd unit files, env-file examples, and small launcher
scripts for the Jetson host. The examples are copied into `/etc/tardigrade/`
and the service files are copied into `/etc/systemd/system/` during setup.

See:

```text
docs/prequal_autostart.md
docs/qual_autostart.md
```
