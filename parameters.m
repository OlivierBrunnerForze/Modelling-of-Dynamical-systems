function p = parameters

p.r_m = 0.01; % Motor radius [m]
p.L_m = 0.01; % Motor inductance [H]
p.R_m = 100; % Motor resistance [ohm]
p.J_m = 10e-6; % Motor inertia [kg*m^2]
p.K_m = 0.05; % Motor constant [Nm/A]
p.m = 0.05; % Tool head mass [kg]
p.mu_r = 0.002; % Bearing friction rolling [Ns/m]
p.mu_lx = 0.001; % Bearing friction linear x-direction [Ns/m]
p.mu_ly = 0.001; % Bearing friction linear y-direction [Ns/m]
p.J_eq = p.J_m + p.m*p.r_m^2/2; % Equivalent rotational inertia [kg*m^2]
p.mu_1 = 3*p.mu_r + 0.5*(p.mu_lx+p.mu_ly); % Equivalent friction 1 [Ns/m]
p.mu_2 = p.mu_r + 0.5*(p.mu_lx - p.mu_ly); % Equivalent friction 2 [Ns/m]
p.m_eq = 2*p.J_m/p.r_m^2 + p.m; % Equivalent mass [kg]
p.b_x = 8*p.mu_r + 2*p.mu_lx; % Equivalent friction x [Ns/m]
p.b_y = 4*p.mu_r + 2*p.mu_ly; % Equivalent friction y [Ns/m]

end
