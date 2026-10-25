%% Q3.2 Draw a square
% Draws a 40 x 40 mm square with corners at (80,80), (120,80), (120,120), (80,120) mm.
% Each edge is one pass trough mode PRINT, the guard 'corner' switches to the next edge
% when the print head reached the corner and the motors stopped. After edge 4 --> DONE
clear; close all; clc;
[p, h] = parameters;
h.job = 'square';

% Initial state: PRINT, edge 1, at rest in the first corner
d0 = struct('mode', 1, 'edge', 1);                       % discrete state PRINT
th = [h.C(1,1) + h.C(1,2), h.C(1,1) - h.C(1,2)]/p.r_m;   % inverse kinematics (Q1.1)
x0 = [th, 0, 0, 0, 0, h.T_print, h.u_hold, h.u_hold/p.R_n, 0, 100, 0, th]'; % initial state

[t, X, M, ev] = simulate(x0, d0, 6, h, p); % hybrid simulation for 6 s
disp(ev)                                   % print list of events

%% Plots
[xp, yp] = kinematics(X(:,1), X(:,2), 0, 0, p);
xz = 120; yz = 80;                                       % zoomed corner [mm]
f3 = figure('Units', 'centimeters', 'Position', [2 2 16 12]);
 
% Full square
subplot(2,2,1); hold on; grid on; axis equal;
plot(1e3*h.C(:,1), 1e3*h.C(:,2), 'k--'); plot(1e3*xp, 1e3*yp, 'b', 'LineWidth', 1.2);
plot(xz, yz, 'ro', 'MarkerSize', 6);
xlim([75 125]); ylim([75 125]);
xlabel('x [mm]'); ylabel('y [mm]'); title('Square');
 
% Zoomed corner
subplot(2,2,3); hold on; grid on; axis equal;
plot(1e3*h.C(:,1), 1e3*h.C(:,2), 'k--'); plot(1e3*xp, 1e3*yp, 'b', 'LineWidth', 1.2);
xlim(xz + [-0.04 0.04]); ylim(yz + [-0.04 0.04]); xticks(xz + [-0.04 0 0.04]); yticks(yz + [-0.04 0 0.04]);
xlabel('x [mm]'); ylabel('y [mm]'); title(sprintf('Corner (%d, %d) mm', xz, yz));
legend('ideal', 'simulated', 'Location', 'northwest');
 
% Motor angles
subplot(2,2,[2 4]); plot(t, X(:,1), t, X(:,2), 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('[rad]'); legend('\theta_l', '\theta_r', 'Location', 'southoutside');
title('Motor angles');
 
set(findall(f3, '-property', 'FontSize'), 'FontSize', 10);
exportgraphics(f3, 'q32_square.pdf', 'ContentType', 'vector');