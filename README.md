# ur3_cup_ball

## Robotics/Sensors Startup Instructions

1. Create catkin workspace
2. Install ur-driver from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
   Follow install instructions on git page
3. Turn on UR3 and wait for it to fully boot
4. Complete required robot calibration as outlined on the git page..
5. start ur-driver with provided script in the robot directory of this repository
6. Put cup-detect pkg in catkin workspace. Build package and then run launch file to start cup-detect
7. Start gui in matlab with gui = GUI();
8. start the ur_ros_driver on the ur3 in order to recieve commands.

## Control For Mx Systems Startup Instructions

1. Create catkin workspace
2. Clone down this repository and initialise git submodules to obtain our forks of the ros_control and universal_robot repositories.
3. Link or copy the cup_detect, jig_description, matlab, ros_control, universal_robot and world sim directories to your catkin workspace
4. Build the full workspace. catkin build from catkin_tools was used for this project
5. Launch the world.launch file from world_sim with ```roslaunch world_sim world.launch``` This will start gazebo with all the models and the cup detection code
6. Unpause the gazebo simultaion to start the physics engine and start the camera sensor
7. Start the main.m file through MATLAB from the matlab directory
8. Move the cup to the desired position with gazebo and then use the matlab gui to calculate trajectories with the calc traj button and then send them to the robot with the execute button.
9. (Optional) Use the suite of RQT tools to either view the bounding_image topic to inspect where the cup is belived to be based on camera data or use the dynamic reconfigure tool to tune PID gains for each joint within the arm controller.

## Requirements
 
NOTE: The GUI will not complete startup unless the cup detect node is running and publishing data. This is because the robot transofrm is required for correct plots i n the GUI.

NOTE: (REAL ROBOT ONLY) without the ur_ros_driver running the robot will not respond to published actions.

NOTE: (REAL ROBOT ONLY) The robot may perform a safety stop during operation. THsi will not clear the trajectory and the robot will try to continue moving once the driver is resumed. the safest course of action is to stop the driver and reload from scratch.

## Sensors and Control Video

The attatched video contains a recorded demonstration of the UR3 making a shot and is followed up by the group quickly describing the process and hardware used in the project.

https://www.youtube.com/watch?v=W9oAOd8Ru0Q

## Contributions

Contribution to Code:
Nicholas Polivis - 33%
Jesse McNamara - 33%
Lucien Morey - 33%

Contribution to Robotics Video 1:
Nicholas Polivis - 33%
Jesse McNamara - 33%
Lucien Morey - 33%

Contribution to Robotics Video 2:
Nicholas Polivis - 33%
Jesse McNamara - 33%
Lucien Morey - 33%

Contribution to Sensors and Control Video:
Nicholas Polivis - 33%
Jesse McNamara - 33%
Lucien Morey - 33%

Contribution to Control for Mechatronic Systems:
Lucien Morey - 50%
George - 50%
