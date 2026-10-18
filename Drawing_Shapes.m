%Model the path of the tool head based on a provided input signal

%Clear workspace, command window text and close all open figure windows
clear; close all; clc      

%Import functions
p = parameters;
[A,  B ]          = dynamics(p);
[Ax, Bx, Ay, By]  = dynamics_decoupled(p);

%Input signal time
T  = 5;                    % simulation time [s]
t  = linspace(0, T, 5001)';     %Time column vector
ts = 2*pi*t/T;
% N    = 2000;                    % samples per lap
% laps = 2;                       % number of laps to draw
% Tl = 5;         %time per lap
% dt = linspace(0, Tl, N+1);
% th   = dt*2*pi/N;

%Define shapes
%Ellipse with major axis 5 cm, minor axis 3 cm
shapes(1).name = 'Ellipse';
shapes(1).X    = 50e-3*cos(2*pi*t/T);
shapes(1).Y    = 30e-3*sin(2*pi*t/T);

% Ellipse
w   = 2*pi/T;
dX1 = -50e-3*w*sin(w*t);   d2X1 = -50e-3*w^2*cos(w*t);   d3X1 =  50e-3*w^3*sin(w*t);
dY1 =  30e-3*w*cos(w*t);   d2Y1 = -30e-3*w^2*sin(w*t);   d3Y1 = -30e-3*w^3*cos(w*t);

% Constants for the model inversion
cm  = 2*p.K_m/(p.r_m*p.m_eq);   %equivalent motor coefficient; acceleration due to current (F_j = 2Km/rm*i_j)
alx = p.b_x/p.m_eq;             % Normalized friction coefficient x
aly = p.b_y/p.m_eq;             % Normalized friction coefficient y

% Required current and voltage per axis (inversion of decoupled model)
ix = (d2X1 + alx*dX1)/cm;
iy = (d2Y1 + aly*dY1)/cm;
ux = p.L_m*(d3X1 + alx*d2X1)/cm + p.R_m*ix + p.K_m/p.r_m*dX1;
uy = p.L_m*(d3Y1 + aly*d2Y1)/cm + p.R_m*iy + p.K_m/p.r_m*dY1;

% Motor voltages (ux = (ul+ur)/2, uy = (ul-ur)/2)
ul = ux + uy;
ur = ux - uy;

%Define decoupled state space model and simulate for x(0)=(0,0,0)
sys_x = ss(Ax, Bx, eye(3), 0);      %output the state vector
sys_y = ss(Ay, By, eye(3), 0);
% Xx = lsim(sys_x, ux, t, zeros(3,1));
% Xy = lsim(sys_y, uy, t, zeros(3,1));
Xx = lsim(sys_x, ux, t, [shapes(1).X(1); dX1(1); ix(1)]);
Xy = lsim(sys_y, uy, t, [shapes(1).Y(1); dY1(1); iy(1)]);

%plots
figure('Name','decoupled input response','Position',[100 100 1100 700]);

subplot(2,2,1)
plot(Xx(:,1)*1e3, Xy(:,1)*1e3, 'k--', 'LineWidth', 1.0)
axis equal; grid on
xlabel('x [mm]'); ylabel('y [mm]')
legend('decoupled','Location','best')
title('Tool head path')

subplot(2,2,2)
plot(t, ux, t, uy, t, ul, '--', t, ur, '--', 'LineWidth', 1.2); grid on
xlabel('t [s]'); ylabel('voltage [V]')
legend('u_x','u_y','u_l','u_r','Location','best')
title('Input signals')

subplot(2,2,3)
plot(t, Xx(:,1)*1e3, 'r--', t, Xy(:,1)*1e3, 'b--')
grid on; xlabel('t [s]'); ylabel('position [mm]')
legend('x decoupled','y decoupled','Location','best')
title('Position vs time')

subplot(2,2,4)
plot(t, Xx(:,2)*1e3, 'r--', t, Xy(:,2)*1e3, 'b--')
grid on; xlabel('t [s]'); ylabel('velocity [mm/s]')
legend('x decoupled','y decoupled','Location','best')
title('Velocity vs time')