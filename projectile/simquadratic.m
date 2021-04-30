clear;clc;

% create projectile
p = Projectile(0.0027, 0.04, 0.5, 0.86);
p1 = Projectile(0.0027, 0.04, 0.55, 0.86);

% initial velocity
alpha = 30;
v  = 0.8;
vi = [v*cosd(alpha), v*sind(alpha)];

% initial position
xi = [0, 1];

% get simu
[x, y, t] = p.simulatep(xi, vi, 3);
figure(1);
plot(x, y);
hold on;
[x, y, t] = p1.simulatep(xi, vi, 3);
plot(x, y);



grid on;
title('Quadratic Resistance - Position x,y');
xlabel('X (m)');
ylabel('Y (m)');
daspect([1 1 1]);
set(gcf,'color','w');
xLimits = get(gca,'XLim');
yLimits = get(gca,'YLim');
xticks(0:0.2:xLimits(end));
yticks(0:0.2:yLimits(end));
legend('Quad resistance');

