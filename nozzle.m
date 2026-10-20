function xdot = nozzle(x, u, p)
    Tn = x(1);  i = x(2);  v = x(3);

    dTn = (v^2/p.R_n - (Tn - p.Te)/p.R_tn) / p.Ct;
    di = (u - v) / p.Ln;
    dv = (i - v/p.Rn) / p.Cn;

    xdot = [dTn; di; dv];
    end