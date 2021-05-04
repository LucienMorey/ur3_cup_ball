clc; close all; clear;
robot = UR5(false);
traj = TrajectoryGenerator(robot.model, [0.2,0,0], 0.3,0.1,[0,0,0]);

[qMatrix,vMatrix,tMatrix] = traj.GenerateTrajectory([0,0,1]);

figure()
robot.model.plot(qMatrix,'trail','r-')