# Pathing Behavior Tree

This document describes the first RoboSub pathing behavior tree for Tardigrade.
The Groot-loadable XML lives at:

```text
behavior_trees/pathing_mission.xml
```

It is a design artifact for now. The node names are intended to become
BehaviorTree.CPP nodes later, with ROS 2 adapters that talk through robot-level
topics and services.

## Mission Shape

The XML is intentionally flattened so Groot shows the main course sequence in
one view instead of hiding the task details behind collapsed subtrees. The
current tree models this course logic:

1. Wait for operator start.
2. Verify robot status, localization, perception, and external-control readiness.
3. Enable external control and arm.
4. Spin-search until the gate is visible.
5. Align with the gate.
6. Choose which side to pass through:
   - `survey_and_repair`
   - `search_and_rescue`
7. Drive through and clear the gate.
8. For each of three slalom markers:
   - search until the marker is visible,
   - align with the marker's left side,
   - pass through on that left-side line.
9. Stop, disable external control, disarm, and report success.

## High-Level Diagram

```mermaid
flowchart TD
    A["Mission Supervisor<br/>ReactiveFallback"] --> B["Abort Path"]
    A --> C["Run RoboSub Pathing Mission"]

    B --> B1["AbortRequested?"]
    B1 --> B2["StopMotion"]
    B2 --> B3["DisarmRobot"]
    B3 --> B4["Report aborted"]

    C --> D["01 Preflight And Enable"]
    C --> E["02 Gate Search, Align, Pass"]
    C --> F1["03 Slalom Marker 1 Left Side"]
    F1 --> F2["04 Slalom Marker 2 Left Side"]
    F2 --> F3["05 Slalom Marker 3 Left Side"]
    F3 --> G["06 Mission Complete"]

    D --> D1["WaitForOperatorStart"]
    D1 --> D2["RobotStatusHealthy?"]
    D2 --> D3["LocalizationHealthy?"]
    D3 --> D4["PerceptionHealthy?"]
    D4 --> D5["ExternalControlReady?"]
    D5 --> D6["SetExternalControl true"]
    D6 --> D7["WaitForExternalControl"]
    D7 --> D8["SetArmed true"]
    D8 --> D9["WaitForArmed"]

    E --> E1["FindGate"]
    E1 --> E2["GateVisible?"]
    E2 -->|No| E3["SpinSearch"]
    E3 --> E1
    E2 -->|Yes| E4["AlignWithGate"]
    E4 --> E5["ChooseGateSide"]
    E5 --> E6["PassThroughGate"]
    E6 --> E7["ClearGate"]

    E5 --> S1["Operator side if available"]
    E5 --> S2["Vision choice fallback<br/>survey_and_repair preferred"]

    F1 --> L1["Find marker 1"]
    L1 --> L2["Align left side"]
    L2 --> L3["Pass marker 1"]

    F2 --> M1["Find marker 2"]
    M1 --> M2["Align left side"]
    M2 --> M3["Pass marker 2"]

    F3 --> N1["Find marker 3"]
    N1 --> N2["Align left side"]
    N2 --> N3["Pass marker 3"]

    G --> G1["StopMotion"]
    G1 --> G2["SetExternalControl false"]
    G2 --> G3["DisarmRobot"]
    G3 --> G4["Report success"]
```

## Runtime Contracts

These are the intended ROS-level contracts for the future implementation.

`AbortRequested`
: Returns success if the operator, watchdog, leak detector, kill switch, or
  mission supervisor requests an abort.

`RobotStatusHealthy`
: Reads `/tardigrade/status` and fails if PX4 is disconnected, arming checks are
  not ready, or a serious status detail is present.

`LocalizationHealthy`
: Checks `/tardigrade/state/odometry` freshness and sanity. This should catch
  stale ZED pose, bad odometry jumps, or impossible position estimates before
  arming/pathing.

`PerceptionHealthy`
: Confirms the camera/perception stack is publishing fresh detections or fresh
  frames before the mission starts.

`GateVisible`
: Returns success once gate perception has a detection above `min_confidence`.

`SpinSearch`
: Publishes a slow yaw command while the gate is not visible.

`AlignWithGate`
: Uses gate centerline/pose to yaw and translate until the robot is aligned.

`OperatorGateSideAvailable`
: Returns success if the operator has already selected the gate side.

`UseOperatorGateSide`
: Writes the operator-selected side into the `gate_side` blackboard value.

`SelectGateSideFromVision`
: Chooses `survey_and_repair` or `search_and_rescue` from perception. The XML
  currently prefers `survey_and_repair` and falls back to `search_and_rescue`.

`PassThroughGate`
: Publishes forward motion through the selected gate side.

`ClearGate`
: Continues forward after the gate so the vehicle is safely past the structure.

`SlalomMarkerVisible`
: Returns success when the requested slalom marker index is visible.

`SweepSearch`
: Runs a short yaw/search pattern while a slalom marker is not visible.

`AlignWithSlalomLeftSide`
: Aligns the robot to pass on the marker's left-side line with a configurable
  lateral offset.

`DrivePastSlalomMarker`
: Drives forward past the requested marker while holding the left-side alignment.

`SetExternalControl`
: Calls `/tardigrade/set_external_control`.

`SetArmed` and `DisarmRobot`
: Call `/tardigrade/set_armed`.

## Blackboard Values

```text
gate_side       survey_and_repair or search_and_rescue
marker_index    slalom marker number, currently 1, 2, or 3
```

## Implementation Notes

The first real code package should probably be `tardigrade_behavior_tree`. It
can wrap BehaviorTree.CPP and communicate through existing robot-level APIs:

```text
/tardigrade/status
/tardigrade/state/odometry
/tardigrade/cmd_vel
/tardigrade/set_external_control
/tardigrade/set_armed
```

Perception nodes should publish robot-level gate and slalom detections. The
behavior tree should orchestrate mission logic; `tardigrade_px4` should remain
the hardware adapter.
