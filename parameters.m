function [p, h] = parameters

%Parameter of the Core XY 3D printer model inspired by the following components (links provided in report):
% Motor      : NEMA 17 17HS4401 (datasheet values)
% Pulley     : GT2 (123-3D)
% Tool head  : E3D Prusa V6 mk3 (24 V / 40 W)

% Motor
p.r_m  = 6.37e-3;             % Pulley/ motor radius [m]
p.L_m  = 2.8e-3;              % Motor induction [H]
p.R_m  = 1.5;                 % Motor resistance [ohm]
p.J_m  = 54e-7;               % Motor inertia [kg*m^2]
p.K_m  = 0.40/1.5;            % Motor constant [Nm/A] (tau=K_m/i_m)

% Tool head (E3D V6, 24 V / 40 W)
p.m      = 0.026;             % Tool head mass [kg]

% nozzle
p.u_n = 24;                 % Nozzle voltage [V]
p.w_n = 40;                 % Nozzle power [W]
p.R_n = p.u_n^2/p.w_n;      % Nozzle resistance [ohm]
p.L_n = 2.8e-3;             % Nozzle inductance [H] (Assumed the same as motor inductance)
p.i_n = p.w_n/p.u_n;        % Nozzle current [A]
p.C_n = 100e-6;             % Nozzle capacitor [F] (Assumption)
p.C_tn = 14.44;             % Nozzle thermal capacity [J/K] (from 20 degC to 200degC in 65 s at 40W)
p.R_tn = 10;                % Nozzle thermal resistance [K/W] (Assumption)

% Friction
p.mu_r  = 0.002;              % Bearing friction rolling [Ns/m]
p.mu_lx = 0.001;              % Bearing friction linear x-direction [Ns/m]
p.mu_ly = 0.001;              % Bearing friction linear y-direction [Ns/m]

% Filament (PLA)
p.rho = 1240;                 % Filament density [kgm^-3]
p.cf = 2000;                  % Filament specific heat capacity [JKg^-1K^-1] 
p.kf = 0.15;                  % Filament thermal conductance [Wm^-1K^-1]
p.w = 1e-3;                   % Filament width [m] (Assumption)

% Additional constants
p.Tb = 333.3;                 % Print bed temperature [K] (60degC for PLA)
p.Te = 293.3;                 % Environmental temperature [K]
p.he = 10.45;                 % Heat transfer coefficient of air [Wm^-2K^-1]
p.hp = 100;                   % Heat transfer coefficient of print bed [Wm^-2K^-1] (Assumption)

% Derived parameters (unchanged formulas)
p.J_eq = p.J_m + p.m*p.r_m^2/2;           % Equivalent rotational inertia [kg*m^2]
p.mu_1 = 3*p.mu_r + 0.5*(p.mu_lx+p.mu_ly);% Equivalent friction 1 [Ns/m]
p.mu_2 = p.mu_r + 0.5*(p.mu_lx - p.mu_ly);% Equivalent friction 2 [Ns/m]
p.m_eq = 2*p.J_m/p.r_m^2 + p.m;           % Equivalent mass [kg]
p.b_x  = 8*p.mu_r + 2*p.mu_lx;            % Equivalent friction x [Ns/m]
p.b_y  = 4*p.mu_r + 2*p.mu_ly;            % Equivalent friction y [Ns/m]

%% Hybrid parameters for Q3
h.Kp      = 2000;              % position gain [V/m]
h.U_move  = 8;                 % voltage limit for travel moves [V]
h.U_sq    = 2;                 % voltage limit while drawing the square [V]
h.eps_th  = 0.002;             % angle tolerance [rad] (~0.01 mm)
h.eps_w   = 0.1;               % "motor stopped" below this speed [rad/s]
h.T_print = 273.15 + 210;      % printing temperature [K]
h.T_low   = 273.15 + 200;      % nozzle temperature low [K]
h.u_hold  = sqrt(p.R_n*(h.T_print - p.Te)/p.R_tn);  % heater voltage that holds T_print [V]
h.T_wipe  = 3;                 % wiping time [s] 
h.T_load  = 5;                 % loading time [s] 
h.t_C     = 5;                 % new colour requested at [s]
h.r0      = 20;                % printing time on the first spool [s]
h.r_new   = 8;                 % printing time on new spools [s]
h.xc = 0.10; h.yc = 0.10; h.a = 0.05; h.b = 0.03; h.w = 2*pi/5;  % ellipse of Q1.4 for Q3.1
h.C  = [0.08 0.08] + 0.04*[0 0; 1 0; 1 1; 0 1; 0 0];           % square corners for Q3.2

end
