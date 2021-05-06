clear;clc;cla;

% create projectile
p = Projectile(0.0027, 0.04, 0.5, 0.86);

% initial velocity
alpha = 30;
v  = 0.8;
vi = [v*cosd(alpha), v*sind(alpha)];

% initial position
xi = [0, 0.4];

% get simu
[x, y, t] = p.simulatep(xi, vi, 4);
fig = figure(1);
h = plot(x, y);

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
plot(0.6, 0.1, 'ro');
legend('Quad resistance', 'Goal pos');

initSlider(h, p);