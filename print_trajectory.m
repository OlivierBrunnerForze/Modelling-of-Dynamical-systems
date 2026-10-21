function traj = print_trajectory(shape, p, t)

syms tau real

X = shape.X;   Y = shape.Y;

% Symbolic derivatives
dX  = diff(X,tau);   d2X = diff(X,tau,2);   d3X = diff(X,tau,3);
dY  = diff(Y,tau);   d2Y = diff(Y,tau,2);   d3Y = diff(Y,tau,3);

% Constants for simplification
cm  = 2*p.K_m/(p.r_m*p.m_eq);   % equivalent motor coefficient (F_j = 2Km/rm*i_j)
alx = p.b_x/p.m_eq;             % normalized friction coefficient x
aly = p.b_y/p.m_eq;             % normalized friction coefficient y

% Inverting the state space to derive currents and input voltages
ix_s = (d2X + alx*dX)/cm;
iy_s = (d2Y + aly*dY)/cm;
ux_s = p.L_m*(d3X + alx*d2X)/cm + p.R_m*ix_s + p.K_m/p.r_m*dX;
uy_s = p.L_m*(d3Y + aly*d2Y)/cm + p.R_m*iy_s + p.K_m/p.r_m*dY;

% Convert symbolic tau back to normal t for numeric functions:
fux = matlabFunction(ux_s, 'Vars', tau);
fuy = matlabFunction(uy_s, 'Vars', tau);
traj.ux  = fux(t) .* ones(size(t));   % use .* ones(size(t)), as ux_s being a constant would return 
% a 1x1 scalar which will be rejected by lsim
traj.uy  = fuy(t) .* ones(size(t));

% Motor voltages
traj.ul = traj.ux + traj.uy;
traj.ur = traj.ux - traj.uy;

% Initial conditions: 
fX  = matlabFunction(shape.X, 'Vars', tau);
fdX = matlabFunction(dX,         'Vars', tau);
fiX = matlabFunction(ix_s,        'Vars', tau);
traj.x0x = [fX(0); fdX(0); fiX(0)];

fY  = matlabFunction(shape.Y, 'Vars', tau);
fdY = matlabFunction(dY,         'Vars', tau);
fiY = matlabFunction(iy_s,        'Vars', tau);
traj.x0y = [fY(0); fdY(0); fiY(0)];

% Evaluate numerically
traj.name = shape.name;
traj.t    = t;
end