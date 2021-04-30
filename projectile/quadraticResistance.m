clear;
cla;
clc;

%% SETUP
% Coefficient of Restitution
% This effects how the velocity is preserved on bounces
% Will need to play with this value because it varies with surface
COR = 0.85;

% diameter of ping-pong ball
% 40mm
d = 0.0040;

% Mass of the pingpong ball
% 2.7g
m = 0.0027;

% accelleration due to gravity
g = 9.81;

% estimate of Cd=0.5 for pingpong balls, could be 0.4-0.5
cd = 5;

% Calculate drag constant - used for calculating force on pingpong ball
k = 0.5 * 1.2754 * cd * pi * d^2/4;

% Number of bounces to calculate
nbounce = 2;

% Terminal velocity, used in time calcs
vterm = sqrt(m * g / k);

%% Calculate trajectory
% Initial projectile motion
[x_air, y_air, vx_air, vy_air, t] = projectile(0, 1, 0.5, 0.3, k, m);

for i = 1:nbounce
    % Calculate new initial velocity/position after bounce
    % This is used for the second stage of motion
    vx0_air = COR * vx_air(end);
    vy0_air = -COR * vy_air(end);
    x0_air = x_air(end);
    y0_air = y_air(end);

    % second bounce
    [x_air2, y_air2, vx_air2, vy_air2, t2] = projectile(x0_air, y0_air, vx0_air, vy0_air, k, m);

    % add movements together
    x_air = [x_air; x_air2];
    y_air = [y_air; y_air2];
    vx_air = [vx_air; vx_air2];
    vy_air = [vy_air; vy_air2];
end
%% plot
figure(1);
plot(x_air, y_air);

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

% anonymous functions

% y direction upwards
% +ve y is upwards
y_up_t = @(t, c) vterm^2/g * log(cos(g*t / vterm)) + c;
vy_up_t = @(t, c) vterm * tan(-g*t / vterm) + c;

% y direction downwards
% +ve y is downwards, so sometimes initial x/v will be negative
y_down_t = @(t, c) vterm * tan(g*t / vterm) + c;
vy_down_t = @(t, c) vterm^2/g * log(cosh(g*t / vterm)) + c;

% x direction
x_t = @(t, c) c / (1 + t * k*c/m);
vx_t = @(t, cv, cx) m/k * log(1 + k*cv/m * t) + cx;

% time functions
% rise time
t1 = @(v0) vterm/g * atan(v0/vterm);

% time to ground
% +ve y is assumed downwards, so initial x0 must be negative.
t2 = @(x0) vterm/g * acosh(exp(-x0*g/vterm^2));


%% Functions
function [x_, y_, vx_, vy_, t] = projectile(x0, y0, vx0, vy0, k, m)
    t = 0;
    dt = 0.001;
    g = 9.81;

    x = x0;
    y = y0;
    vx = vx0;
    vy = vy0;

    x_ = [];
    y_ = [];
    vx_ = [];
    vy_ = [];
  
    while y >= 0
        x_ = [x_; x];
        y_ = [y_; y];
        vx_ = [vx_; vx];
        vy_ = [vy_; vy];
        
        ay = -g + -k/m * vy^2 * sign(vy);
        ax = -k/m * vx^2 * sign(vx);

        vx = vx + ax*dt;
        vy = vy + ay*dt;
        
        x = x + vx*dt;
        y = y + vy*dt;
        
        t = t + dt;
    end
end


