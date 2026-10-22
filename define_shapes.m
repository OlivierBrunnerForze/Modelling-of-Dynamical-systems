function shapes = define_shapes(T, t)
% Here T is the period of the shape drawn and 
% t is the time vector. Use for example t  = linspace(0, T, 5001)';

w = 2*pi/T;                 % angular frequency of one drawing [rad/s]
syms tau real

% Ellipse with semi-major axis 5 cm, semi-minor axis 3 cm
shapes(1).name = 'Ellipse';
shapes(1).X    = 0.05*cos(w*tau);
shapes(1).Y    = 0.03*sin(w*tau);

% Spade
shapes(2).name = 'Spade';
shapes(2).X    = 16*sin(w*tau)^3*2e-3;
shapes(2).Y    = -2e-3*(13*cos(w*tau)-5*cos(2*w*tau)-2*cos(3*w*tau)-2*cos(4*w*tau));

%circle with radius 5 cm
shapes(3).name = 'Circle';
shapes(3).X = 0.05*cos(w*tau);
shapes(3).Y = 0.05*sin(w*tau);


% Shift such that x and y are never negative (defined in Q1.1)
for k = 1:numel(shapes)
    fXk = matlabFunction(shapes(k).X, 'Vars', tau);
    fYk = matlabFunction(shapes(k).Y, 'Vars', tau);
    shapes(k).X = shapes(k).X - min(fXk(t) .* ones(size(t)));
    shapes(k).Y = shapes(k).Y - min(fYk(t) .* ones(size(t)));
end

end