% no bounce, angle calc
clear;clc;cla;close all;

g = 9.81;

% create projectile
p = Projectile(0.0027, 0.04, 0, 0.86);

% goal pos
h = 0.08;
d = 0.40;

% initial position
xi = [0, 0.4];

% initial velocity magnitude
v0 = 1;

% solve for theta
c = d - xi(1);
q1 = v0^2/ (g*c);
q2 = v0^2 * (v0^2 - 2*g*h + 2*g*xi(2)) / (g * c)^2;
theta = atan( q1 + sqrt(q2 - 1) );
theta = solveQuad(xi, 1, d, h, 0.86);

% get initial velocity
vi = v0*[cos(theta), sin(theta)];

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