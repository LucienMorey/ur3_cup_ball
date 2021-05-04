clc; close all; clear;
robot = UR5(false);
trajgen = TrajectoryGenerator(robot.model, transl(0.1,0.1,0.2), 0.1,0.1, transl(-0.1,0.1,0.2));
[qMatrix,vMatrix,tMatrix] = trajgen.GenerateTrajectory([0;0;1]);
% hold on;
% robot.model.animate(qMatrix(1,:));
% plot3(trajgen.traj(1,:), trajgen.traj(2,:), trajgen.traj(3,:));
% 
% for i = 1:1:size(trajgen.orientation,3)
%     trplot(trajgen.orientation(:,:,i), 'length', 0.05);
% end
% figure()
% robot.model.plot(qMatrix, 'trail', 'r');