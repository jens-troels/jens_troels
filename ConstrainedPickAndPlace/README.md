# Example code for constrained pick and place task with a UR robot, using constrained pathplanning and pose estimation

This code does not work without the right cmake files, the right catkin workspace for ROS and the right library for controlling the UR robot. This code should be purely seen as inspiration for future projects and guidance in difficult problems

Overview of the project is as follows:
- Code to control the gripper used for gripping the object after its position has been verified by pose estimation
- Pose estimation done by an intel stereo camera to get the real world position of the object
- Pathplanning and optimization of the path taken from the robotics start position and to the detected object
- General robot control
- ROS integration to make the all the above points work together
