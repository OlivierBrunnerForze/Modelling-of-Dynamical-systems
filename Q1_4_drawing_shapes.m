% Model the path of the tool head based on a provided input signal
clear; close all; clc

p = parameters;

% Time
T = 5;                          % simulation time [s]
t = linspace(0, T, 5001)';      % numeric time column vector

% ---- Names of the shapes to draw ----
names = {'Ellipse', 'Spade', 'Circle'};   % any list of names defined in define_shapes
% names = {'Spade'};            % just one
% names = {define_shapes(T,t).name};   % all defined shapes

for k = 1:numel(names)
    [traj, sim] = run_shape(names{k}, p, T, t);

    figure('Name', ['decoupled input response: ' traj.name], 'Position', [100 100 1100 700]);

    subplot(2,2,1)
    hP = plot(sim.Xx(:,1)*1e3, sim.Xy(:,1)*1e3, 'k--', 'LineWidth', 1.0);
    hold on
    hS = plot(sim.Xx(1,1)*1e3,   sim.Xy(1,1)*1e3,   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    hE = plot(sim.Xx(end,1)*1e3, sim.Xy(end,1)*1e3, 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r');
    hold off
    axis equal; grid on
    xlabel('x [mm]'); ylabel('y [mm]')
    legend([hP hS hE], {'decoupled','start','end'}, 'Location','best')
    title(['Tool head path: ' traj.name])

    subplot(2,2,2)
    plot(t, traj.ux, t, traj.uy, t, traj.ul, '--', t, traj.ur, '--', 'LineWidth', 1.2); grid on
    xlabel('t [s]'); ylabel('voltage [V]')
    legend('u_x','u_y','u_l','u_r','Location','best')
    title('Input signals')

    subplot(2,2,3)
    plot(t, sim.Xx(:,1)*1e3, 'r--', t, sim.Xy(:,1)*1e3, 'b--')
    grid on; xlabel('t [s]'); ylabel('position [mm]')
    legend('x decoupled','y decoupled','Location','best')
    title('Position vs time')

    subplot(2,2,4)
    plot(t, sim.Xx(:,2)*1e3, 'r--', t, sim.Xy(:,2)*1e3, 'b--')
    grid on; xlabel('t [s]'); ylabel('velocity [mm/s]')
    legend('x decoupled','y decoupled','Location','best')
    title('Velocity vs time')
end