clc; close all; clear;

robot = UR3m(trotz(-pi/2));

q = [0 -pi/2 -pi/6 -pi/4 0 0];

start = [0.2, 0.2, 0.5];
goal  = [0.85, -0.25, 0.1];
reload = [-0.3,0.1,0.4];

reload_position = transl(reload(1), reload(2), reload(3));
throw_position = transl(start(1), start(2), start(3)); 
startQ = robot.model.ikcon(reload_position, [0, -pi, -pi/2, -pi/2, 0, 0]);

robot.model.animate(startQ);
view(22, 15);

trajgen = TrajectoryGenerator(robot.model, throw_position, 0.25, 0.1, reload_position);
n = 1;
v0 = 1;
p = Projectile(0.0027, 0.04, 0, 0.86);
v = p.calcLaunch(start, goal, n, v0);
xyz = p.simulateP(start, v, 1);
[qMatrix,vMatrix,tMatrix, xMatrix] = trajgen.GenerateThrow(v);

hold on;
plot3(xyz(:, 1), xyz(:, 2), xyz(:,3));              % ball path
plot3(goal(1), goal(2), goal(3), 'ro');             % goal pos
plot3(start(1), start(2), start(3), 'mo');          % start pos
plot3(reload(1), reload(2), reload(3), 'bo'); % reload pos

% animatethe robot
robot.model.plot(qMatrix, 'trail', 'r', 'fps', 50);
plot3(xMatrix(:, 1), xMatrix(:, 2), xMatrix(:, 3), 'k'); % robot traj xyz

grid on;
title('Trajectory 2D x,y');
xlabel('X (m)');
ylabel('Y (m)');
daspect([1 1 1]);
set(gcf,'color','w');
pos = get(gca, 'OuterPosition');
set(gca, 'OuterPosition', pos);
legend('Ball Path', 'Robot path', 'Goal', 'Start', 'Reload');
%gui = GUI();







