%% Question 1 Physical Modelling

p = parameters;
[A, B] = dynamics(p);

format shortG 
disp(A); disp(B); disp(eig(A))