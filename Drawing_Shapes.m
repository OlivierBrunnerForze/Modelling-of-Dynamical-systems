%Model the path of the tool head based on a provided input signal

%Clear workspace, command window text and close all open figure windows
clear; close all; clc      

%Import functions
p = parameters;
[A,  B ]          = dynamics(p);
[Ax, Bx, Ay, By]  = dynamics_decoupled(p);

%Input signal time
T  = 2;                    % simulation time [s]
t  = linspace(0, T, 2001)';     %Time column vector

% Define a smooth two-axis input trajectory
[ul, ur] = deal(sin(2*pi*t/T), cos(2*pi*t/T));
ux = 1/2 * (ul + ur);
uy = 1/2 * (ul - ur);


%Define decoupled state space model and simulate for x(0)=(0,0,0)
sys_x = ss(Ax, Bx, eye(3), 0);      %output the state vector
sys_y = ss(Ay, By, eye(3), 0);
Xx = lsim(sys_x, ux, t, zeros(3,1));
Xy = lsim(sys_y, uy, t, zeros(3,1));

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