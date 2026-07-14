# Legacy Packages

Pixhawk/PX4 code is currently preserved under:

```text
src/legacy/tardigrade_px4
src/legacy/px4_msgs
```

`src/legacy/COLCON_IGNORE` keeps these packages out of normal colcon discovery,
so the ESP-first workspace stays light:

```bash
./build.sh
```

To intentionally work on the legacy path, remove `src/legacy/COLCON_IGNORE` or
move the packages back into the active workspace.
