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

Legacy automatic prequalification and qualification startup assets remain in:

```text
robot/autostart/
```

Do **not** install or enable them for the current pool stack. They predate the
final Jetson-controller/binary-ESP path and are retained only until autonomy is
rebuilt on the same controller-enable contract as assisted teleop. Use
`docs/pool_teleop.md` for current robot operation.
