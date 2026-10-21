% Solution to Q1.6; printing a circle of filament (PDE) 
% This script aims to solve the model of the heat transfer in the filament
% numerically, using ss and lsim

clear; close all; clc
p = parameters;

% Setting up print geometry and discretisation
Rc   = 5e-3;              % circle radius [m]  
L    = 2*pi*Rc;           % loop length [m]
N    = 200;               % number of blocks
ds   = L/N;               % block length [m]
Tpr  = 5;                 % time to print the circle [s]
vn   = L/Tpr;             % For circle we assume constant nozzle speed [m/s]
Af   = p.w^2;               % cross-section [m^2]

% Temperatures
Tn = 200 + 273.15;        % nozzle temperature [K] (for simplicity of this model we will give it a constant value here)
Tp = p.Tb;                % plate temperature [K]
Te = p.Te;                % environment temperature [K]
u  = [Tp; Te];

% Block constants 
Ct = p.rho*p.cf*Af*ds;    % heat capacity of one block [J/K]
kc = p.kf*Af/ds;          % conduction between neighbouring blocks [W/K]
kp = p.hp*p.w*ds;           % heat transfer block to plate [W/K]
ke = 3*p.he*p.w*ds;         % heat transfer block to environment [W/K]
par = struct('Ct',Ct,'kc',kc,'kp',kp,'ke',ke);          % printing parameters (for functions)

% Time
tk      = (0:N-1)*ds/vn;  % activation times
nsam    = 6;                          % samples per segment (including both end points)
ncol    = N*(nsam-1) + 1;             % total number of columns while printing
tg1     = zeros(1, ncol);             % grid of sample times (while printing)
Tmap1   = nan(N, ncol);               % NaN means block not printed yet (doesn't have a temperature)

% Setting up settling criteria (blocks sufficiently close to equilibrium temperature)
Teq    = (p.hp*Tp + 3*p.he*Te)/(p.hp + 3*p.he);   % eq. (1.63)
tol    = 5;               % blocks should be within 1 degC of equilibrium temperature
frac   = 0.95;            % fraction of blocks that must satisfy tolerance
t_max  = Tpr + 300;       % maximum duration of simulation [s]

% Printing filament before loop is closed. One block added per step
T = [];
for k = 1:N
    t0 = tk(k);
    if k < N, t1 = tk(k+1); else, t1 = Tpr; end   % t1 = time the next block is deposited
    T = [T; Tn];                                  % new block starts at nozzle temperature

    [Am, Bm] = build_AB(k, false, par);           %obtain A and B for state space model
    sys  = ss(Am, Bm, eye(k), 0);                 % output is temperatures of all k blocks
    tseg = linspace(t0, t1, nsam)';               % time vector for each block/segment
    Y    = lsim(sys, repmat(u', nsam, 1), tseg, T);   % simulate state space model for time vector tseg

    cols = (k-1)*(nsam-1) + (1:nsam);             % match columns with specific block (shares first column with previous one)
    tg1(cols)        = tseg;                      % store time for each blcok
    Tmap1(1:k, cols) = Y';                        %store time per block
    T = Y(end,:)';                                % last sample of block is initial state of next
end

% closing the loop and run under closed loop conditions. Run until most blocks are near Teq
[Am, Bm] = build_AB(N, true, par);
sys   = ss(Am, Bm, eye(N), 0);
Nt2   = 3001;                                     % number of samples for time after loop is closed
t_rel = linspace(0, t_max - Tpr, Nt2)';           % time vector after closing the loop
Y     = lsim(sys, repmat(u', Nt2, 1), t_rel, T);  % simulate closed loop system

% Find the first sample where most blocks are close enough to the
% equilibrium temperature
d      = sort(abs(Y - Teq), 2);     %sort from smallest to largest deviation
dq     = d(:, ceil(frac*N));        % index frac*N is the frac*N th smallest deviation 
i_stop = find(dq < tol, 1);         % if true means that enough blocks satisfy tolerance
if isempty(i_stop)
    warning('Settling criterion not reached before max simulation time');
    i_stop = Nt2;
end

% Append both phases (printing and closed loop cooldown)
tg2   = Tpr + t_rel(1:i_stop)';
Tmap2 = Y(1:i_stop, :)';                          
tg    = [tg1, tg2(2:end)];
Tmap  = [Tmap1, Tmap2(:,2:end)];                  % Rows are blocks 0 to N-1, columns are sample times (tg)
t_stop = tg(end);

fprintf('%.0f%% of blocks within %.1f K of Teq (%.2f K) after t = %.1f s (%.1f s after closing)\n', ...
        100*frac, tol, Teq, t_stop, t_stop - Tpr);

% Plots
th = 2*pi*(0:N-1)'/N;
xc = Rc*cos(th)*1e3;  yc = Rc*sin(th)*1e3;          % block positions [mm]

figure('Name','Filament temperature','Position',[100 100 1100 450]);
% subplot(1,2,1)
pcolor(tg, 1:N, Tmap - 273.15); shading flat; colorbar
xline(Tpr,'w--','t_{close}');
xlabel('t [s]'); ylabel('block index i'); title('Temperature [°C]')

figure('Name','Selected blocks','Position',[100 100 600 400]);
sel = round(linspace(1, N, 5));
plot(tg, Tmap(sel,:) - 273.15, 'LineWidth', 1.2); hold on
yline(Teq-273.15, 'k--', 'T_{eq}'); hold off; grid on
xlabel('t [s]'); ylabel('T [°C]')
legend(arrayfun(@(i) sprintf('block %d', i-1), sel, 'UniformOutput', false), 'Location','best')
title('Temperature of selected blocks')

% Local functions
function [A, B] = build_AB(m, closed, par)
    % Linear model Tdot = A T + B u 
    Lap = -2*eye(m) + diag(ones(m-1,1),1) + diag(ones(m-1,1),-1);
    if closed
        Lap(1,m) = 1;  Lap(m,1) = 1;                % when loop is closed T_{-1}=T_{N-1}, T_N=T_0
    else
        Lap(1,1) = Lap(1,1) + 1;                    % T_{-1} = T_0     (no heat flow out from first block)
        Lap(m,m) = Lap(m,m) + 1;                    % T_{m}  = T_{m-1} (no heat flow out from filament at the ends)
    end
    A = (par.kc*Lap - (par.kp + par.ke)*eye(m)) / par.Ct;       % Transform A to include actual heat transfer terms
    B = ones(m,1) * [par.kp, par.ke] / par.Ct;                  % Transform B to include actual heat transfer terms
end