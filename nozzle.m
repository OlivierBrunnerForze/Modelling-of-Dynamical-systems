function xdot = nozzle(x, u, p)
%Nozzle model with x = [Tn; uc; iL] and input u = u_n (nozzle voltage)    
    Tn = x(1);  i = x(2);  v = x(3);

    dTn = (v^2/p.R_n - (Tn - p.Te)/p.R_tn) / p.C_tn;
    di = (u - v) / p.L_n;
    dv = (i - v/p.R_n) / p.C_n;

    xdot = [dTn; di; dv];
    end