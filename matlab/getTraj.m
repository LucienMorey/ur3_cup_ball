%{
    This function is an intermediary between the projectile object and the
    GUI.

    TASKS -> recieve start/end/plothandle
          -> Create projectile, calculate trajectory
          -> Plot trajectory in 3D
          -> Plot trajectory in 2D
          -> returns the 3D velocity vector so it can be published in the
          GUI

    START -> should be the hardcoded launch point for the ball
    GOAL  -> x,y,z of cup top centre
%}

function v = getTraj(start, goal, h1, h2)

    % create projectile
    p = Projectile(0.0027, 0.04, 0, 0.86);

    % velocity magnitude, this is just constant for now
    v0 = 1;

    % number of bounces
    n = 1;
    
    % get launch vel
    [v, xi, d, h, theta] = p.calcLaunch(start, goal, n, v0);

    % get initial velocity for plot
    vi = v0 * [cos(theta), sin(theta)];

    % 2D simulation of projectile
    [x, y, t] = p.simulatep(xi, vi, 1);
    
    % plot the 2D one
    set(h2, 'xdata', x);
    set(h2, 'ydata', y);
    plot(h2, d, h, 'ro');
    legend('Trajectory', 'Goal pos');
    
    % TODO: PLOT3d

end