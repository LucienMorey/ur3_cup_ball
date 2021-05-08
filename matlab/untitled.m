clc; close all; clear;

robot = UR3m();
pause(2);

reload_position = transl(-0.1,0,0.55);
throw_position = transl(0,0.3,0.3);
  
startQ = robot.model.ikcon(reload_position, ones(1,6));

robot.model.animate([0, 0, 0, 0, 0, 0]);

trajgen = TrajectoryGenerator(robot.model, throw_position, 0.1,0.45, reload_position);
[qMatrix,vMatrix,tMatrix] = trajgen.GenerateThrow([1; 0; 0]);

robot.model.plot(qMatrix, 'trail', 'r', 'fps', 40);

[qMatrix,vMatrix,tMatrix] = trajgen.jog(qMatrix(end, :), [0 0.3 0]);
robot.model.plot(qMatrix, 'trail', 'r', 'fps', 10);
%gui = GUI();
