function sim = simulate_trajectory(traj, p)

%Import functions
[Ax, Bx, Ay, By] = dynamics_decoupled(p);

sys_x = ss(Ax, Bx, eye(3), 0);      % output the full state vector
sys_y = ss(Ay, By, eye(3), 0);

sim.Xx = lsim(sys_x, traj.ux, traj.t, traj.x0x);
sim.Xy = lsim(sys_y, traj.uy, traj.t, traj.x0y);

end