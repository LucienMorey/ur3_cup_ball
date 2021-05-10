clc; close all; clear;

robot = UR3m(trotz(pi/2));

q = [0 -pi/2 -pi/6 -pi/4 0 0];

reload_position = transl(-0.1,0,0.55);
throw_position = transl(0,0.2,0.3); 
startQ = robot.model.ikcon(reload_position, ones(1,6));

robot.model.animate(q);
view(22, 15);

trajgen = TrajectoryGenerator(robot.model, throw_position, 0.3, 0.3, reload_position);
n = [1; 0; 0];
n = n/norm(n);
[qMatrix,vMatrix,tMatrix] = trajgen.GenerateThrow(n);

robot.model.plot(qMatrix, 'trail', 'r', 'fps', 30);

% pause(1.5);
% [qMatrix,vMatrix,tMatrix] = trajgen.cjog(q, [0.2 0 0]);
% robot.model.plot(qMatrix, 'trail', 'r', 'fps', 10);
% 
% pause(0.5);
% [qMatrix,vMatrix,tMatrix] = trajgen.cjog(qMatrix(end, :), [0 0.2 0]);
% robot.model.plot(qMatrix, 'trail', 'r', 'fps', 10);
% 
% pause(0.5);
% [qMatrix] = trajgen.jjog(qMatrix(end, :), [2.4 -0.7 0 0.6 0.8 0]);
% robot.model.plot(qMatrix, 'trail', 'r', 'fps', 10);

%gui = GUI();
