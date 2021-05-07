clc; close all; clear;

robot = UR5();

reload_position = transl(0.4,0,0.4);
throw_position = transl(0,0.4,0);
  
startQ = robot.model.ikcon(reload_position, ones(1,6));

figure()
robot.model.animate([0, 0, 0, 0, 0, 0]);

trajgen = TrajectoryGenerator(robot.model, throw_position, 0.3,0.5, reload_position);
[qMatrix,vMatrix,tMatrix] = trajgen.GenerateTrajectory([1; 0; 0]);

robot.model.plot(qMatrix, 'trail', 'r');



%gui = GUI();
