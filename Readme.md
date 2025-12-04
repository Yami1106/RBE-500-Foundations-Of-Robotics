## Prerequisites

```bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
---
```

## Part 1: Velocity Kinematics

```bash
ros2 run RBE_500_final velocity_kinematics
```

**Test 1: Joint velocities → End-effector velocities**
```bash
ros2 service call /joint_vel_to_ee_vel interfaces_pkg/srv/JointVeltoEEVel \
"{q: [0.0, 0.0, 0.0, 0.0], q_dot: [0.1, 0.1, 0.0, 0.0]}"
```
- `q`: joint positions (rad)
- `q_dot`: joint velocities (rad/s)
- Returns: `x_dot` (6 values: linear velocity [m/s] + angular velocity [rad/s])

**Test 2: End-effector velocities → Joint velocities**
```bash
ros2 service call /ee_vel_to_joint_vel interfaces_pkg/srv/EEVeltoJointVel \
"{q: [0.0, 0.0, 0.0, 0.0], x_dot: [x_dot values that we get from 1]}"
```
- `q`: current joint positions (rad)
- `x_dot`: desired end-effector velocity (6 values: [vx, vy, vz, wx, wy, wz])
- Returns: `q_dot` (4 joint velocities in rad/s)

---

## Part 2: Incremental Joint Positions

**Start the node:**
```bash
ros2 run RBE_500_final incremental_joint_positions
```

**Monitor output:**
```bash
ros2 topic echo /incremental_joint_targets
```

**Command joint velocities:**
```bash
ros2 topic pub /incremental_qdot_cmd std_msgs/msg/Float64MultiArray \
"{data: [0.1, 0.0, 0.0, 0.0]}" --once
```
- `data`: 4 joint velocities (rad/s)
- Watch `/incremental_joint_targets` - joint positions should increment

---

## Part 3: Linear Motion

**Terminal 1: Velocity kinematics**
```bash
ros2 run RBE_500_final velocity_kinematics
```

**Terminal 2: Incremental joint positions**
```bash
ros2 run RBE_500_final incremental_joint_positions
```

**Terminal 3: Linear motion controller**
```bash
ros2 run RBE_500_final linear_motion
```
- Commands robot to move at 0.03 m/s in +Y direction

**Terminal 4: Forward kinematics (not necessary)**
```bash
ros2 run RBE_500_final fwd_kinematics
```

**Monitor Y position:**
```bash
ros2 topic echo /tool_pose --field position.y
```
- Y coordinate should increase over time

**Record data for report:**
```bash
ros2 topic echo /tool_pose > cartesian_trajectory.txt
```
- Run for 30 seconds, then Ctrl+C
- Plot Y vs time for report

---

## Quick Check Commands

```bash
# Check joint velocities being commanded
ros2 topic echo /incremental_qdot_cmd

# Check joint positions being sent
ros2 topic echo /incremental_joint_targets

# Check end-effector pose
ros2 topic echo /tool_pose
```