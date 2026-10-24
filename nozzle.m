function xdot = nozzle(x, u, p)
% Nozzle model with x = [Tn; uc; iL] and input u = u_n (nozzle voltage)
Tn = x(1);  uc = x(2);  iL = x(3);

dTn = (uc^2/p.R_n - (Tn - p.Te)/p.R_tn) / p.C_tn;   % nozzle temperature
duc = (iL - uc/p.R_n) / p.C_n;                      % capacitor
diL = (u - uc) / p.L_n;                             % inductor

xdot = [dTn; duc; diL];
end
