clc; close all; clear;
% robot = UR5(false);
% reload_position = transl(0,0,0.4);
% throw_position = transl(0.2,0.2,0.2);
% startQ = robot.model.ikcon(reload_position, ones(1,6));
% robot.model.animate(startQ);
% trajgen = TrajectoryGenerator(robot.model, throw_position, 0.1,0.1, reload_position);
% [qMatrix,vMatrix,tMatrix] = trajgen.GenerateTrajectory([0;0.5;0.5]);
% figure()
% hold on;
% plot3(trajgen.cartesianTrajectory(1,:), trajgen.cartesianTrajectory(2,:), trajgen.cartesianTrajectory(3,:));
% 
% for i = 1:1:size(trajgen.cartesianWaypoints,3)
%     trplot(trajgen.cartesianWaypoints(:,:,i), 'length', 0.05);
% end
% figure()he
% robot.model.plot(qMatrix, 'trail', 'r');


gui = GUI();