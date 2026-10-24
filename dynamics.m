function [A, B] = dynamics(p)

A = [0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, -p.r_m^2*p.mu_1/p.J_eq, -p.r_m^2*p.mu_2/p.J_eq, p.K_m/p.J_eq, 0;
     0, 0, -p.r_m^2*p.mu_2/p.J_eq, -p.r_m^2*p.mu_1/p.J_eq, 0, p.K_m/p.J_eq;
     0, 0, -p.K_m/p.L_m, 0, -p.R_m/p.L_m, 0;
     0, 0, 0, -p.K_m/p.L_m, 0, -p.R_m/p.L_m];

B = [0, 0;
     0, 0;
     0, 0;
     0, 0;
     1/p.L_m, 0;
     0, 1/p.L_m];

end
