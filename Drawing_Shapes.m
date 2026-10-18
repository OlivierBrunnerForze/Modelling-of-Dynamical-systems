%Model the path of the tool head based on a provided input signal
%Clear workspace, command window text and close all open figure windows
clear; close all; clc

%Import functions
p = parameters;
[A,  B ]          = dynamics(p);
[Ax, Bx, Ay, By]  = dynamics_decoupled(p);

%Input signal time
T  = 5;                         % simulation time [s]
t  = linspace(0, T, 5001)';     % numeric time column vector
w  = 2*pi/T;                    %Frequency domain

%setting up symbolic function for MATLAB's derivative function
syms tau real                   % symbolic variable for time (otherwise it will interfere with t)

%Define shapes
%Ellipse with major axis 5 cm, minor axis 3 cm
shapes(1).name = 'Ellipse';
shapes(1).X    = 0.05*cos(w*tau);
shapes(1).Y    = 0.03*sin(w*tau);

%Spade
shapes(2).name = 'Spade';
% shapes(2).X = sin(w*tau) * (1 + cos(w*tau)) *(1.2 + 0.3*cos(5*w*tau));
% shapes(2).Y = cos(w*tau) * (1 + cos(w*tau)) - 0.5*sin(w*tau)^2*cos(2*w*tau);
shapes(2).X = 16*sin(w*tau)^3*2e-3;
shapes(2).Y = -2e-3*(13*cos(w*tau)-5*cos(2*w*tau)-2*cos(3*w*tau)-2*cos(4*w*tau));

%Shift such that x and y can never be negative which was defined in Q1.1:
for k = 1:length(shapes)
    fXk = matlabFunction(shapes(k).X, 'Vars', tau);
    fYk = matlabFunction(shapes(k).Y, 'Vars', tau);
    shapes(k).X = shapes(k).X - min(fXk(t));
    shapes(k).Y = shapes(k).Y - min(fYk(t));
end

for k=1:length(shapes)
    % Setting up derivatives using MATLAB's derivative function and symbolic
    % variable tau
    
    dX1 = diff(shapes(k).X,tau);   d2X1 = diff(shapes(k).X,tau,2);   d3X1 = diff(shapes(k).X,tau,3);
    dY1 = diff(shapes(k).Y,tau);   d2Y1 = diff(shapes(k).Y,tau,2);   d3Y1 = diff(shapes(k).Y,tau,3);
    
    % Constants for simplification
    cm  = 2*p.K_m/(p.r_m*p.m_eq);   % equivalent motor coefficient (F_j = 2Km/rm*i_j)
    alx = p.b_x/p.m_eq;             % normalized friction coefficient x
    aly = p.b_y/p.m_eq;             % normalized friction coefficient y
    
    % Inverting State Space to derive input signal: 
    ix_s = (d2X1 + alx*dX1)/cm;
    iy_s = (d2Y1 + aly*dY1)/cm;
    ux_s = p.L_m*(d3X1 + alx*d2X1)/cm + p.R_m*ix_s + p.K_m/p.r_m*dX1;
    uy_s = p.L_m*(d3Y1 + aly*d2Y1)/cm + p.R_m*iy_s + p.K_m/p.r_m*dY1;
    
    % Convert symbolic tau back to normal t for numeric functions:
    fux = matlabFunction(ux_s, 'Vars', tau);
    fuy = matlabFunction(uy_s, 'Vars', tau);
    ux  = fux(t) .* ones(size(t));   % use .* ones(size(t)), as ux_s being a constant would return 
                                     % a 1x1 scalar which will be rejected by lsim
    uy  = fuy(t) .* ones(size(t));
    
    % Motor voltages
    ul = ux + uy;
    ur = ux - uy;
    
    % Initial conditions: 
    fX  = matlabFunction(shapes(k).X, 'Vars', tau);
    fdX = matlabFunction(dX1,         'Vars', tau);
    fiX = matlabFunction(ix_s,        'Vars', tau);
    x0x = [fX(0); fdX(0); fiX(0)];
    
    fY  = matlabFunction(shapes(k).Y, 'Vars', tau);
    fdY = matlabFunction(dY1,         'Vars', tau);
    fiY = matlabFunction(iy_s,        'Vars', tau);
    x0y = [fY(0); fdY(0); fiY(0)];
    
    %Define decoupled state space model and simulate
    sys_x = ss(Ax, Bx, eye(3), 0);      % output the state vector
    sys_y = ss(Ay, By, eye(3), 0);
    
    Xx = lsim(sys_x, ux, t, x0x);
    Xy = lsim(sys_y, uy, t, x0y);
    
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
end