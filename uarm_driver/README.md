This document describes the ROS topics used to control the CR45 robotic arm.

# Setup
Run in first terminal:
```bash
roslaunch swiftpro pro_control.launch
```
Start rosCore and launches serial_write script

# SwiftPro ROS Topics

Each topic acts as an action trigger: when a new message is published, the corresponding action is executed.

---

## Gripper Control

**Topic:** `/gripper_topic`
**Message Type:** `swiftpro/status`
**Behavior:** Controls the gripper state.

| `status` | Action           |
|----------|----------------|
| 0        | Open gripper    |
| 1        | Close gripper   |

*Publishing a new message triggers the gripper action immediately.*

---

## Gripper Rotation

**Topic:** `/angle4th_topic`
**Message Type:** `swiftpro/angle4th`
**Field:** `angle4th` (float)
**Behavior:** Rotates the gripper around its fourth axis.

Example:
- `angle4th: 180.0` → Rotates the gripper to 180 degrees.

*Rotation is triggered upon receiving a new message.*

---

## End-Effector Position Control

**Topic:** `/position_write_topic`
**Message Type:** `geometry_msgs/Point`
**Fields:**
- `x` (float)
- `y` (float)
- `z` (float)

**Behavior:** Moves the robot's end-effector to the specified coordinates.

Example:
```yaml
x: 150.00
y: 150.00000013
z: 130.00


