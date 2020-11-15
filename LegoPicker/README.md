# Scattered example code for a project involving picking and packing lego pieces from specific order. Uses a UR robot and a MiR robot to complete the task.

This project does not work without the right catkin workspace, library for the UR robot (URTDE), the right cmake files. The code in this project should only be seen as inspiration for future project. Positions in the workcell are also hardcoded according to the prefered positions during execution.

Overview of the project is as follows:
- General control of the robot (UR robot arm)
- Control of the MiR robot
- MES (to grab packing orders from an online server)
- Raspberry Pi module with a connected module to use Computer Vision to distinguish correct lego pieces
- Modbus protocol for communication between Pi and Robot
- ROS integration for communication between everything else
- Control of a light tower placed in the corner of the work cell
