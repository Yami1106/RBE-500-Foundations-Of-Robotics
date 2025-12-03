## Execution

1) example for vel-kinematics : q here is the home position, can be changed, a_dot is for the joint velocities

```bash
ros2 service call /joint_vel_to_ee_vel interfaces_pkg/srv/JointVeltoEEVel \
"{q: [0.0, 0.0, 0.0, 0.0], q_dot: [0.1, 0.1, 0.0, 0.0]}"
```

2) End effector velocity to joint velocities
```bash
ros2 service call /ee_vel_to_joint_vel interfaces_pkg/srv/EEVeltoJointVel \
"{q: [0.0, 0.0, 0.0, 0.0], x_dot: [0.0, 0.01, 0.0, 0.0, 0.0, 0.0]}"
```



3) incremental positions

```bash 
ros2 topic pub /incremental_qdot_cmd std_msgs/Float64MultiArray \
"{data: [0.1, 0.0, 0.0, 0.0]}"
```
```bash
ros2 topic echo /incremental_joint_targets
```

4) linear motion in +y direction : 

ros2 run RBE_500_final linear_motion

