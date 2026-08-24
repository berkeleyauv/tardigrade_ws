# Running The Simulator

End-to-end steps to drive the robot in the Unity pool sim (or the ROS-only
fake backend), from a cold machine. If you only want to change a path, the
short version is [Quick Start in local_sim_backend.md](local_sim_backend.md#quick-start-change-a-path-and-run-it);
this doc is the full runbook including Docker, Unity, teleop, and common
problems.

There are two backends and everything upstream is identical between them:

- **Fake backend** (`fake_unity_backend`): ROS-only, no Unity. Fastest to
  test mission/path logic.
- **Unity backend** (`RosRobotBridge` in `tardigrade_unity_sim`): the actual
  pool sim. Connects over `ros_tcp_endpoint`.

---

## 0. One-time / cold start

**Start Docker Desktop** (Windows) and wait for the whale icon to settle.

**Start the container.** If it already exists but is stopped:

```bash
docker start tardigrade-foxy
```

If it doesn't exist yet (first time, or after removing it), from the repo
root:

```bash
cd "C:\Users\elila\Desktop\Robosub Stuff\tardigrade_ws"
./docker-build.sh --detached
```

Confirm it's up:

```bash
docker ps --filter name=tardigrade-foxy
```

> The container stops every time Docker Desktop restarts. `docker start
> tardigrade-foxy` brings it back with the workspace already built — you do
> **not** need to rebuild the image each time.

**Build the ROS workspace** — only needed the first time, or after changing a
`.msg`/`.srv` or adding a new node/entry point. Plain edits to existing Python
files need no rebuild (see the rebuild table in
[local_sim_backend.md](local_sim_backend.md#does-this-need-a-rebuild)):

```bash
docker exec -it tardigrade-foxy bash
cd /ws && ./build.sh
exit
```

---

## 1. Start a backend (Terminal 1)

Every terminal that talks to ROS needs the same two lines first:

```bash
docker exec -it tardigrade-foxy bash
cd /ws && source install/setup.bash
```

Then, **pick one backend:**

### Option A — Fake backend (no Unity)

```bash
ros2 run tardigrade_sim fake_unity_backend
```

### Option B — Unity

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

Then open the `tardigrade_unity_sim` project in Unity and press **Play**.
Check the Console for `[RosRobotBridge] up. sub /tardigrade/cmd_vel; ...`.
Confirm **Robotics → ROS Settings** is `ROS2` / `127.0.0.1` / `10000`.

Leave Terminal 1 running.

---

## 2. Arm the robot (Terminal 2)

The robot ignores all commands until external control is enabled **and** it's
armed. Do this once per session:

```bash
docker exec -it tardigrade-foxy bash
cd /ws && source install/setup.bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Both should respond `success: True`.

> `path_follower` (below) arms and disarms itself, so you can skip this step
> if you're **only** running `path_follower`. You need it for teleop.

---

## 3. Drive it

### Teleop (keyboard)

In Terminal 3 (own interactive shell — must be `docker exec -it`):

```bash
docker exec -it tardigrade-foxy bash
cd /ws && source install/setup.bash
ros2 run tardigrade_teleop keyboard_cmd_vel
```

Click into that terminal so it has keyboard focus, then press keys (no Enter):

```text
w/s  forward / back
j/l  strafe left / right
r/f  up / down
a/d  yaw left / right (turns in place)
space  stop
Ctrl-C  quit
```

### Follow a path (waypoints)

In Terminal 3 instead:

```bash
docker exec -it tardigrade-foxy bash
cd /ws && source install/setup.bash
ros2 run tardigrade_mission path_follower
```

Default is a 3 m square. Custom path / speed / depth:

```bash
ros2 run tardigrade_mission path_follower --ros-args \
  -p waypoints:="[2.0, 0.0, 2.0, 2.0, 0.0, 2.0]" \
  -p forward_cmd:=0.4 -p target_depth_m:=1.0
```

Full parameter list and how to write a different path shape:
[local_sim_backend.md → Driving A Custom Path](local_sim_backend.md#driving-a-custom-path).

---

## 4. Watch it move (optional Terminal 4)

With Unity, just watch the Scene/Game view. Either way you can echo state:

```bash
docker exec -it tardigrade-foxy bash
cd /ws && source install/setup.bash
ros2 topic echo /tardigrade/state/odometry
```

---

## 5. Shut down

- `Ctrl-C` in each ROS terminal.
- If you were teleoping, disarm: re-run the `set_armed` call from step 2 with
  `{armed: false}`.
- The container can stay running; it's cheap. Stop it with `docker stop
  tardigrade-foxy` if you want.

---

## Troubleshooting

**`docker exec ... container ... is not running`** — Docker Desktop restarted
and stopped the container. `docker start tardigrade-foxy`.

**`failed to connect to the docker API ...`** — Docker Desktop itself isn't
running. Start it, wait for the tray icon to settle.

**Unity: `Connection to 127.0.0.1:10000 failed ... connection was aborted`** —
usually the very first Play after a container restart; just press Play again.
If it persists, confirm the endpoint (step 1B) is running and ROS Settings is
`ROS2`.

**Laggy / delayed movement in Unity (Windows + Docker Desktop)** — commands
take many seconds (observed ~45s) to show up in Unity even though they're being
published fine. This is **antivirus / firewall real-time inspection of Docker
Desktop's network traffic on Windows**, not a ROS or code problem. The tells:

- The same workspace is fast on native Linux Docker (no VM) and on macOS with
  OrbStack (leaner network stack) — both bypass Docker Desktop's Windows
  network proxy. Only Windows + Docker Desktop is slow.
- ROS discovery (`ros2 topic list`, `node list`) works, and the intermittent
  `connection was aborted by the software in your host machine` error on the
  first Play is the same fingerprint: something Windows-side is inspecting and
  interfering with the connection.

Fix: in **Windows Security → Virus & threat protection → Manage settings →
Exclusions**, add folder exclusions for `C:\Program Files\Docker`,
`%LOCALAPPDATA%\Docker`, and the Unity install folder, plus process exclusions
for `Docker Desktop.exe`, `com.docker.backend.exe`, `com.docker.vpnkit.exe`,
and `Unity.exe`. Then restart Docker Desktop (tray icon → Restart) and
`docker start tardigrade-foxy` again. If a third-party AV is installed, add the
same exclusions there.

If it's still slow after exclusions: bump Docker Desktop's CPU/memory in
Settings → Resources, or run Docker Engine natively inside a WSL2 distro
instead of through Docker Desktop (removes the Windows proxy layer entirely,
matching the fast Linux/OrbStack setups; Unity still connects to
`127.0.0.1:10000` unchanged).

**Robot moves forward too slowly** — increase teleop speed:
`ros2 run tardigrade_teleop keyboard_cmd_vel --ros-args -p linear_step:=0.8`,
or `forward_cmd` for `path_follower`.

**Robot swings/arcs instead of turning in place (Unity)** — the robot's FBX
pivot is off-center. Fix once by centering the pivot: **save and close Unity**,
then run

```bash
"/c/Program Files/Unity/Hub/Editor/6000.2.6f2/Editor/Unity.exe" -batchmode -nographics -quit \
  -projectPath "C:/Users/elila/Desktop/Robosub Stuff/tardigrade_unity_sim" \
  -executeMethod CliCenterPivot.CenterRobotPivot
```

then reopen Unity. See `Assets/Editor/CliCenterPivot.cs` in the Unity repo.

**Keys do nothing in teleop** — did you arm (step 2)? Is the terminal an
interactive `-it` shell with keyboard focus? Watch `/tardigrade/cmd_vel` with
`ros2 topic echo /tardigrade/cmd_vel` to confirm keys are being published.
