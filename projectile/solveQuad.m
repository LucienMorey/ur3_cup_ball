% CASE: 1 bounce, no air resistance
% Try to solve for theta (a) using optimisation solver


% x0 -> initial position [x, y] in-plane
% v0 -> initial velocity [x, y] in-plane
% d  -> horizontal displacement from launch->goal in-plane
% h  -> height of target (cup)
% e  -> coefficient of resitution
function res = solveQuad( x0, v0, d, h, e)
    % const
    g = 9.81;

    % intemediate value
    c1 = d - x0(1);

    % Function of theta
    fun = @(a) e*sqrt( v0^2*sin(a)^2 + 2*g*x0(2) )*(c1/(v0*cos(a)) - (v0*sin(a) + sqrt( v0^2*sin(a)^2 + 2*g*x0(2) ))/g ) - 0.5*g*(c1/(v0*cos(a)) - (v0*sin(a) + sqrt( v0^2*sin(a)^2 + 2*g*x0(2) ))/g )^2 - h;

    % Attempt to solve for theta, with initial guess of pi/4
    res = fsolve(fun, pi/4);
end