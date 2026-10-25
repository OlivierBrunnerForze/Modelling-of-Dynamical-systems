function [T, X, M, ev] = simulate(x, d, tf, h, p)
% Hybrid simulation, discrete mode transition if a guard holds,
% otherwise continuous evolution (ODE solver) ubtil a guard becomes true
%
% Continuous state x (14 states):
%   1-6   th_l, th_r, dth_l, dth_r, i_l, i_r    motors + tool head (Q1.2)
%   7-9   T_n, u_c, i_L                         nozzle (Q1.5)
%   10    s                                     printing time (for the position on the ellipse)
%   11    r                                     printing time left on the spool
%   12    tau                                   clock, reset at every mode change
%   13-14 th_ls, th_rs                          saved motor angles
%
% Discrete state d.mode (Moore machine of Q2.1, MOVE split into V and H as in Q2.2):
%   1 PRINT  2 SAVE  3 MOVE_V  4 MOVE_H  5 WIPE  6 HEAT_OFF  
%   7 LOAD 8 HEAT_ON  9 RETURN  10 DONE (end of the square)
%
% Outputs: time T, states X, mode M, list of events ev = {time, name}

t = 0;  T = [];  X = [];  M = [];  ev = {};         % start time and result arrays
while t < tf                                        % simulation time
    [g, names] = guards(t, x, d, h, p);             % evauluate guards of current mode
    k = find(g >= -1e-6, 1);                        % first guard that holds
    if ~isempty(k)                                  % guard holds -> mode transition
        [x, d] = reset_map(names{k}, x, d, h);      % apply reset map
        ev(end+1, :) = {t, names{k}};               % save event time and name
        continue                                    % check guards of the new mode
    end
    opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', 0.02, 'Events', @(tt, xx) event_fun(tt, xx, d, h, p)); % solver settings
    [Ts, Xs] = ode15s(@(tt, xx) sys(xx, inputs(xx, d, h, p), p), [t tf], x, opts);  % integrate dx=sys(x,u)
    T = [T; Ts];  X = [X; Xs];  M = [M; d.mode*ones(size(Ts))];
    t = Ts(end);  x = Xs(end, :)';  % continue where solver stopped
end
end

% All state space descriptions combined in dx = sys(x, u),  u = [u_l; u_r; u_n; p]
function dx = sys(x, u, p)
[A, B] = dynamics(p);
dx = [A*x(1:6) + B*u(1:2);          % motors + tool head (Q1.2)
      nozzle(x(7:9), u(3), p);      % nozzle (Q1.5)
      u(4);                         % dt_p   = 1 while printing
     -u(4);                         % dt_p   = -1 while printing
      1;                            % dtau = 1
      0; 0];                        % saved angles do not change
end

% Inputs per mode (output of Moore machine Q2.1)
function u = inputs(x, d, h, p)
[xp, yp] = kinematics(x(1), x(2), 0, 0, p);      % position from the motor angles
[xs, ys] = kinematics(x(13), x(14), 0, 0, p);    % saved position
printing = 0;
if d.mode == 1 && strcmp(h.job, 'ellipse')       % PRINT: follow the ellipse
    ph = h.w*x(10);  w = h.w;                    % position on the ellipse
    ux = ff(-h.a*w*sin(ph), -h.a*w^2*cos(ph),  h.a*w^3*sin(ph), p.b_x, p) ...
         + h.Kp*(h.xc + h.a*cos(ph) - xp); % from feedforward (Q1.4) and P-controller (Q3.1)
    uy = ff( h.b*w*cos(ph), -h.b*w^2*sin(ph), -h.b*w^3*cos(ph), p.b_y, p) ...
         + h.Kp*(h.yc + h.b*sin(ph) - yp); % from feedforward (Q1.4) and P-controller (Q3.1)
    printing = 1;
else                                             % all other modes: go to a target point
    switch d.mode
        case 1,          P = h.C(d.edge + 1, :);  U = h.U_sq;   % square: next corner
        case {2, 9, 10}, P = [xs, ys];            U = h.U_move; % SAVE, RETURN, DONE
        case 3,          P = [xs, 0];             U = h.U_move; % MOVE_V: straight down
        otherwise,       P = [0, 0];              U = h.U_move; % MOVE_H ... HEAT_ON
    end
    ux = sat(h.Kp*(P(1) - xp), U);  % P-controller in x
    uy = sat(h.Kp*(P(2) - yp), U);  % P-controller in y
end
switch d.mode
    case {6, 7}, un = 0;                         % HEAT_OFF, LOAD: heater off
    case 8,      un = p.u_n;                     % HEAT_ON: heater on (max voltage)
    otherwise,   un = h.u_hold;                  % keep T_n at T_print
end
u = [ux + uy; ux - uy; un; printing];            % u_l = u_x + u_y, u_r = u_x - u_y (Q1.3)
end

% Guards (g(k) >= 0 -> guard k holds)
function [g, names] = guards(t, x, d, h, p)
thl = x(1); thr = x(2); Tn = x(7); r = x(11); tau = x(12);
stopped = h.eps_w - max(abs(x(3)), abs(x(4)));            % event 7: motors stopped
at = @(al, ar) min([h.eps_th - abs(thl - al), h.eps_th - abs(thr - ar), stopped]);
switch d.mode
    case 1   % PRINT
        if strcmp(h.job, 'ellipse')
            gC = min(t - h.t_C, h.t_C - (t - tau));       % C: t reaches t_C (once)
            g = [gC; -r];  names = {'C', 'F'};            % F: spool empty
        else
            P = h.C(d.edge + 1, :);                       % next corner of the square
            g = at((P(1) + P(2))/p.r_m, (P(1) - P(2))/p.r_m);  names = {'corner'};
        end
    case 2,  g = stopped;                                 names = {'s'};    % SAVE: head stopped
    case 3,  g = min(h.eps_th - abs(thl - thr), stopped); names = {'V0'};   % MOVE_V: y = 0
    case 4,  g = at(0, 0);                                names = {'H0'};   % MOVE_H: x = y = 0
    case 5,  g = tau - h.T_wipe;                          names = {'w'};    % WIPE: wipe time passed
    case 6,  g = h.T_low - Tn;                            names = {'o'};    % HEAT_OFF: T_n < T_low
    case 7,  g = tau - h.T_load;                          names = {'l'};    % LOAD: load time passed
    case 8,  g = Tn - h.T_print;                          names = {'h'};    % HEAT_ON: T_n at T_print
    case 9,  g = at(x(13), x(14));                        names = {'r'};    % RETURN: back at saved angles
    otherwise, g = -1;                                    names = {'none'}; % DONE: no transition
end
end

% Reset maps (new mode and state reset for each mode)
function [x, d] = reset_map(name, x, d, h)
x(12) = 0;                                       % clock resets in every new mode
switch name
    case {'C', 'F'}, d.mode = 2;  x(13:14) = x(1:2);    % color change/spool empty -> SAVE, save motor angles
    case 's',        d.mode = 3;                        % stopped -> MOVE_V
    case 'V0',       d.mode = 4;                        % at y=0 -> MOVE_H
    case 'H0',       d.mode = 5;                        % at (0,0) -> WIPE
    case 'w',        d.mode = 6;                        % wiped -> HEAT_OFF
    case 'o',        d.mode = 7;                        % cooled down -> LOAD
    case 'l',        d.mode = 8;  x(11) = h.r_new;      % loaded -> HEAT_ON, new spool print time
    case 'h',        d.mode = 9;                        % hot -> RETURN
    case 'r',        d.mode = 1;                        % at saved point -> PRINT
    case 'corner'
        d.edge = d.edge + 1;                                  % next edge of the square
        if d.edge > 4, d.mode = 10;  x(13:14) = x(1:2); end   % square done
end
end

% Event function for ode: stop when a guard goes from < 0 to >= 0
function [v, term, dir] = event_fun(t, x, d, h, p)
v = guards(t, x, d, h, p);  term = ones(size(v));  dir = ones(size(v));
end

% Feedforward voltage of one axis (Q1.4)
function u = ff(v, a, j, b, p)
cm = 2*p.K_m/p.r_m;
u  = p.L_m*(p.m_eq*j + b*a)/cm + p.R_m*(p.m_eq*a + b*v)/cm + p.K_m/p.r_m*v;
end

% Limit a value to [-lim, lim]
function y = sat(y, lim)
y = min(max(y, -lim), lim);
end