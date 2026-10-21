function [traj, sim] = run_shape(name, p, T, t)

shapes = define_shapes(T, t);

idx = ismember({shapes.name}, char(name));

%run the functions to obtain the tajectories and simulations
traj = print_trajectory(shapes(idx), p, t);
sim = simulate_trajectory(traj, p);
end