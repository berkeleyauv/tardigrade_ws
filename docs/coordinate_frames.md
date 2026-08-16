# Tardigrade Coordinate Frames

## The short version

A frame is a named three-axis ruler. Tardigrade uses the ROS robot-body
convention everywhere after sensor conversion:

```text
base_link +X  robot forward
base_link +Y  robot left
base_link +Z  robot up
```

Positive roll, pitch, and yaw use the right-hand rule around +X, +Y, and +Z.
That means left-side-up is positive roll, nose-up is negative pitch, and
nose-left is positive yaw.

The frame tree for the pool test is:

```text
odom  --dynamic EKF pose-->  base_link
                               |--static mount--> zed_camera_link
                               `--static mount--> vectornav
```

- `base_link` moves with the robot and is the frame used by control.
- `vectornav` and `zed_camera_link` are rigid sensor frames.
- `odom` is a locally continuous world frame. It starts at an arbitrary origin
  and may slowly drift, but it should not jump.
- `map` is a globally corrected world frame. It can move relative to `odom`
  after a localization correction. The pool controller does not need `map`.
- TF is the ROS transform graph that connects these named rulers.

A `nav_msgs/Odometry` message has two important names. `header.frame_id` is the
world/reference ruler and `child_frame_id` is the moving object whose pose is
being reported. Therefore a ZED message with `odom` and `zed_camera_link` means
"the pose of the ZED camera measured in the local odometry world."

## VectorNav convention and installed mounting

VectorNav produces native NED navigation data and FRD sensor-body data. ROS
control expects ENU navigation data and FLU robot-body data. Those convention
changes are different from the physical sensor mounting rotation.

The installed mounting is currently encoded as:

```text
VectorNav printed +X  robot backward
VectorNav printed +Y  robot left
VectorNav printed +Z  robot down
```

This is a 180-degree rotation about robot +Y, quaternion `(x=0, y=1, z=0,
w=0)`. The `vectornav_imu_transform` node performs both the native convention
conversion and this mounting correction once, then publishes
`/tardigrade/sensors/imu` with `frame_id: base_link`. Downstream odometry and
the EKF must use that converted topic and must not convert it again.

The initial measured sensor translations from the geometric center of the
46 x 29 x 22 cm main frame are:

```text
ZED:       x=+0.300 m, y=+0.000 m, z=-0.050 m
VectorNav: x=-0.035 m, y=+0.145 m, z=+0.000 m
```

Here +X is forward toward the ZED, +Y is robot-left, and +Z is up. The ZED
value uses the midpoint between its lenses; the VectorNav value uses the
approximate center of its enclosure.

If visual inspection shows the printed +Y arrow does not point robot-left,
stop: at least one of the stated X/Z directions is also wrong, because the
three printed arrows form a right-handed coordinate system. Correct the four
`vectornav_q*` launch values before using feedback control.

## Safe unpowered verification

Build and source the workspace, then start the ZED without its dynamic TF. The
EKF is the sole owner of `odom -> base_link`:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed publish_tf:=false
```

Start VectorNav and the explicit conversion node:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  use_zed_orientation_if_imu_stale:=false
```

Then start the EKF:

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Check the graph and converted data:

```bash
ros2 run tf2_ros tf2_echo base_link vectornav
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 topic echo /tardigrade/sensors/imu --once
ros2 topic echo /tardigrade/state/odometry/filtered --once
```

Expected results:

- `base_link -> vectornav` reports a quaternion equivalent to `(0, 1, 0, 0)`.
- `base_link -> zed_camera_link` reports approximately
  `(x=0.30, y=0.00, z=-0.05)` metres with identity rotation.
- Converted IMU messages say `frame_id: base_link`.
- With the robot still and level, roll and pitch are stable and near their
  expected level values.
- Nose-left motion produces positive yaw rate.
- Left-side-up motion produces positive roll rate.
- Nose-up motion produces negative pitch rate.
- Moving the robot upward makes ZED/filtered odometry Z increase.

Do not enable an attitude PID if any one of these signs is wrong. A sign error
can make feedback reinforce the disturbance instead of opposing it.
