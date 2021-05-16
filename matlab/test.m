clear;close all;

ur3 = UR3m();

q0 = [0, -pi/2, 0 , 0, 0, 0];
qlist = [q0; [0, 0, 0, 0, 0, 0]]
ur3.model.animate(q0);

p = [0, 0, 0.25];
n = [0, 0, 1];

col = checkCollision(ur3, qlist, p, n)