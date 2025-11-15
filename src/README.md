# OpenManipulatorX_ROS2
ROS2 Humble packages to control Open Manipulator X

## Build
Copy the contents of the entire repository into the src/ directory of a new ROS2 workspace.
```
mkdir -p rbe500_ws/src
cd rbe500_ws
git clone https://github.com/yera217/OpenManipulatorX_ROS2.git ./src
```

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds with no errors.


Don't forget to source your workspace (and add source line with full path to ~/.bashrc if you want to automatically source the workspace for each new terminal instance):
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```


---
RBE 500 - Foundations of Robotics 2025 taught by Professor Haichong Zhang at Worcester Polytechnic Institute Robotics Engineering Department.

Forked from Steven Hyland's repo for RBE500 Fall2023 offering taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department.
