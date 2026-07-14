# Local Sim Backend

This is a ROS-only stand-in for the future Unity scene. It lets us test mission
logic before Unity exists.

## What It Provides

Run:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

It provides:

```text
/tardigrade/status
/tardigrade/state/odometry
/tardigrade/perception/gate
/tardigrade/perception/slalom
/tardigrade/set_external_control
/tardigrade/set_armed
```

It subscribes:

```text
/tardigrade/cmd_vel
```

When armed and external control is enabled, it integrates `/tardigrade/cmd_vel`
into fake odometry. It also publishes fake gate and slalom detections.

## Build

From the local Docker container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  tardigrade_interfaces tardigrade_sim tardigrade_mission
source install/setup.bash
```

## Test The Backend

Terminal 1:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Terminal 2:

```bash
ros2 topic echo /tardigrade/status
ros2 topic echo /tardigrade/state/odometry
ros2 topic echo /tardigrade/perception/gate
```

Terminal 3:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

The fake robot moves only after these services are called:

```bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

## Test The Gate Mission

Terminal 1:

```bash
ros2 run tardigrade_sim fake_unity_backend
```

Terminal 2:

```bash
ros2 run tardigrade_mission gate_mission
```

The mission:

1. waits for fake status,
2. enables external control,
3. arms the fake backend,
4. spins/searches until the gate is visible,
5. aligns to the gate yaw error,
6. drives forward through the gate,
7. stops, disarms, and disables external control.

Unity should later replace `fake_unity_backend` while preserving the same ROS
topics and services.
