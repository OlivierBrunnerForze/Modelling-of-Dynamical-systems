function [Ax, Bx, Ay, By] = dynamics_decoupled(p)
% Decoupled dynamics of x and y, state vector [pos; vel; current]

Ax = [0, 1, 0;
      0, -p.b_x/p.m_eq, 2*p.K_m/(p.r_m*p.m_eq);
      0, -p.K_m/(p.r_m*p.L_m), -p.R_m/p.L_m];
Bx = [0; 0; 1/p.L_m];

Ay = [0, 1, 0;
      0, -p.b_y/p.m_eq, 2*p.K_m/(p.r_m*p.m_eq);
      0, -p.K_m/(p.r_m*p.L_m), -p.R_m/p.L_m];
By = [0; 0; 1/p.L_m];

end
