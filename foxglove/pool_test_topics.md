# Pool Test Topics

This is the first topic plan for the Foxglove pool-test layout. Treat it as a
target interface list; some topics do not exist yet.

## Robot Status

```text
/tardigrade/status
```

Robot-level Pixhawk connection, armed state, external-control state, odometry
freshness, and debug detail.

## State Estimate

```text
/tardigrade/state/odometry
/vectornav/imu
/tf
/tf_static
```

Show pose, orientation, frame tree, and IMU behavior.

## Commands

```text
/tardigrade/cmd_vel
```

Current robot-level velocity command. Useful for checking teleop/autonomy
intent before worrying about physical motion.

## Cameras

Known or expected sources:

```text
/zed/zed_node/left/image_rect_color
/zed/zed_node/right/image_rect_color
/zed/zed_node/depth/depth_registered
```

Open questions:

- What are the exact ZED topics on the Jetson with the current wrapper config?
- What is the final downward Arducam image topic?

## Perception Debug

Target topics:

```text
/tardigrade/perception/gate/detections
/tardigrade/perception/slalom/detections
/tardigrade/perception/debug_image
```

Open question: use `vision_msgs` if it is practical on Foxy, otherwise define
small custom messages in `tardigrade_interfaces`.

## Autonomy

Target topics:

```text
/tardigrade/autonomy/state
/tardigrade/autonomy/events
```

These do not exist yet. Add them when the first mission runner exists.

