# ur3_cup_ball

1. Create catkin workspace
2. Install ur-driver from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
   Follow install instructions on git page
3. Take ur3_launch files from raspberry pi. Run launch file to start ur-driver
4. Put cup-detect in catkin workspace. Run launch file to start cup-detect
5. Start gui in matlab with gui = GUI();

Set the video source
rosparam set /video_device /dev/video0
