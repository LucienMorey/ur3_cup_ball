% try to solve for theta (a) using optimisation solver

function res = solveQuad( x0, v0, d, h, e)
% const
g = 9.81;

c1 = d - x0(1)

fun = @(a) e*sqrt( v0^2*sin(a)^2 + 2*g*x0(2) )*(c1/(v0*cos(a)) - (v0*sin(a) + sqrt( v0^2*sin(a)^2 + 2*g*x0(2) ))/g ) - 0.5*g*(c1/(v0*cos(a)) - (v0*sin(a) + sqrt( v0^2*sin(a)^2 + 2*g*x0(2) ))/g )^2 - h;

res = fsolve(fun, 0.76)
end