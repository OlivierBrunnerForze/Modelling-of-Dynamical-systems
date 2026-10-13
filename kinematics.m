function [x, y, xdot, ydot] = kinematics(theta_l, theta_r, thetadot_l, thetadot_r, p)
% Kinematics of tool head position and velocity from motor angles
% theta in [rad], thetadot in [rad/s], x,y in [m], xdot,ydot in [m/s]

x = p.r_m/2 * (theta_l + theta_r);
y = p.r_m/2 * (theta_l - theta_r);
xdot = p.r_m/2 * (thetadot_l + thetadot_r);
ydot = p.r_m/2 * (thetadot_l - thetadot_r);

end
