# ur3_cup_ball

## Startup Instructions

1. Create catkin workspace
2. Install ur-driver from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
   Follow install instructions on git page
3. Turn on UR3 and wait for it to fully boot
4. Complete required robot calibration as outlined on the git page..
5. start ur-driver with provided script in the robot directory of this repository
6. Put cup-detect pkg in catkin workspace. Build package and then run launch file to start cup-detect
7. Start gui in matlab with gui = GUI();
8. start the ur_ros_driver on the ur3 in order to recieve commands.

## Requirements
 
NOTE: The GUI will not complete startup unless the cup detect node is running and publishing data. This is because the robot transofrm is required for correct plots i n the GUI.

NOTE: without the ur_ros_driver running the robot will not respond to published actions.

NOTE: The robot may perform a safety stop during operation. THsi will not clear the trajectory and the robot will try to continue moving once the driver is resumed. the safest course of action is to stop the driver and reload from scratch.
