clc; close all; clear;

% robot = UR3m(trotz(pi/2));
% 
% q = [0 -pi/2 -pi/6 -pi/4 0 0];
% 
% reload_position = transl(-0.1,0,0.55);
% throw_position = transl(0.2,-0.2,0.45); 
% startQ = robot.model.ikcon(reload_position, ones(1,6));
% 
% robot.model.animate(q);
% view(22, 15);
% 
% trajgen = TrajectoryGenerator(robot.model, throw_position, 0.2, 0.1, reload_position);
% n = [1; 0; 0];
% n = n/norm(n);
% [qMatrix,vMatrix,tMatrix] = trajgen.GenerateThrow(n);
% 
% robot.model.plot(qMatrix, 'trail', 'r', 'fps', 30);

% start = [0.1; 0.1; 0.5];
% goal  = [0.9; 0; 0.1];
% 
% n = 1;
% v0 = 1;
% 
% p = Projectile(0.0027, 0.04, 0, 0.86);
% v = p.calcLaunch(start, goal, n, v0);
% xyz = p.simulateP(start, v, 1);
% 
% plot3(xyz(:, 1), xyz(:, 2), xyz(:,3));
% hold on;
% plot3(goal(1), goal(2), goal(3), 'ro');
% 
% grid on;
% title('Trajectory 2D x,y');
% xlabel('X (m)');
% ylabel('Y (m)');
% daspect([1 1 1]);
% set(gcf,'color','w');
% %xlim([0 1.2]);
% %ylim([0 1.2])
% pos = get(gca, 'OuterPosition');
% set(gca, 'OuterPosition', pos);
gui = GUI();
