% no bounce, angle calc
clear;clc;cla;close all;

% create projectile
p = Projectile(0.0027, 0.04, 0, 0.86);


% start/end
start = [0, 0, 0.4];
goal  = [0.80, 0, 0.08];

% velocity magnitude
v0 = 1;

% get launch vel
[v, xi, d, h, theta] = p.calcLaunch(start, goal, 1, v0);

% get initial velocity for plot
vi = v0 * [cos(theta), sin(theta)];

% get simu
[x, y, t] = p.simulatep(xi, vi, 1);
fig = figure(1);
plot(x, y);

grid on;
title('Quadratic Resistance - Position x,y');
xlabel('X (m)');
ylabel('Y (m)');
daspect([1 1 1]);
set(gcf,'color','w');
xlim([0 1.2]);
ylim([0 1.2])
xLimits = get(gca,'XLim');
yLimits = get(gca,'YLim');
xticks(0:0.2:xLimits(end));
yticks(0:0.2:yLimits(end));
pos = get(gca, 'OuterPosition');
pos(2) = pos(1) + 0.1;
pos(4) = pos(4) - 0.1;
set(gca, 'OuterPosition', pos);

hold on;
plot(d, h, 'ro');
legend('Quad resistance', 'Goal pos');