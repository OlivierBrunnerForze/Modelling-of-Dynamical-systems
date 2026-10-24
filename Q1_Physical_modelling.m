%% Question 1 Physical Modelling

p = parameters;


[A, B] = dynamics(p);
format shortG 
disp(A); disp(B); disp(eig(A))

[Ax, Bx, Ay, By] = dynamics_decoupled(p);
format shortG
disp(eig(Ax)); disp(eig(Ay))