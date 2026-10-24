%% Q3.1 - Filament replacement while printing the ellipse
% Simulates printing of the ellipse from Q1.4. At t = t_C a color change is
% requested and at a later time the spool runs out of filament, both
% triggering a replacement cycle:
% SAVE -> MOVE_V -> MOVE_H -> WIPE -> HEAT_OFF -> LOAD -> HEAT_ON -> RETURN -> PRINT
clear; close all; clc;
[p, h] = parameters;
h.job = 'ellipse';

% Initial state: PRINT, start of the ellipse (x = 150 mm, y = 100 mm), moving upward
d0 = struct('mode', 1);                                % discrete state PRINT
th = [h.xc + h.a + h.yc, h.xc + h.a - h.yc]/p.r_m;     % inverse kinematics (Q1.1)
w  = [h.b*h.w, -h.b*h.w]/p.r_m;                        % motor speeds vx = 0, vy = b*w
x0 = [th, w, 0, 0, h.T_print, h.u_hold, h.u_hold/p.R_n, 0, h.r0, 0, th]'; % initial state

[t, X, M, ev] = simulate(x0, d0, 75, h, p); % hybrid simulation for 75 s [time, states, modes, events]
disp(ev)                                    % print list of events

%% Plots
[xp, yp] = kinematics(X(:,1), X(:,2), 0, 0, p);
xa = xp; ya = yp; xa(M ~= 1) = NaN; ya(M ~= 1) = NaN;   % printing 
xb = xp; yb = yp; xb(M == 1) = NaN; yb(M == 1) = NaN;   % travel 

% Print head path
f1 = figure('Units', 'centimeters', 'Position', [2 2 14 11]);
hold on; grid on; axis equal;
plot(1e3*xa, 1e3*ya, 'b-', 'LineWidth', 1.2);
plot(1e3*xb, 1e3*yb, 'r-', 'LineWidth', 1.2);
xlim([-5 160]); ylim([-5 135]);
xlabel('x [mm]'); ylabel('y [mm]'); legend('printing', 'travel', 'Location', 'northwest');
set(findall(f1, '-property', 'FontSize'), 'FontSize', 10);
exportgraphics(f1, 'q31_path.pdf', 'ContentType', 'vector');

% Modes, position and nozzle temperature
f2 = figure('Units', 'centimeters', 'Position', [2 2 16 14]);
subplot(3,1,1); stairs(t, M, 'k', 'LineWidth', 1); grid on; xlim([0 75]); ylim([0.5 9.5]);
set(gca, 'YDir', 'reverse', 'YTick', 1:9, 'YTickLabel', ...
    {'PRINT','SAVE','MOVE\_V','MOVE\_H','WIPE','HEAT\_OFF','LOAD','HEAT\_ON','RETURN'});
title('Mode');
subplot(3,1,2); plot(t, 1e3*xp, t, 1e3*yp, 'LineWidth', 1); grid on; xlim([0 75]); ylim([-10 160]);
ylabel('[mm]'); legend('x', 'y', 'Location', 'eastoutside'); title('Position (from motor angles)');
subplot(3,1,3); plot(t, X(:,7) - 273.15, 'r', 'LineWidth', 1); grid on; xlim([0 75]); ylim([190 215]);
ylabel('T_n [\circC]'); xlabel('t [s]'); title('Nozzle temperature');
set(findall(f2, '-property', 'FontSize'), 'FontSize', 9);
exportgraphics(f2, 'q31_states.pdf', 'ContentType', 'vector');