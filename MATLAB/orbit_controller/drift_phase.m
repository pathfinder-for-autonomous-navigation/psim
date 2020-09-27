function [t, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift)
% Performs the drift phase of the simulation. Propegates the satellites
% orbits to 't_drift' seconds and plots the position and velocity of
% satellite two relative to satellite one if plot is true.

opt = odeset('RelTol', 1e-5, 'AbsTol', 1e-4);
[t, y] = ode113(@frhs, [0.0, t_drift], [r1; v1; r2; v2], opt);

t = t(end);
r1 = y(end, 1:3)';
v1 = y(end, 4:6)';
r2 = y(end, 7:9)';
v2 = y(end, 10:12)';

end

function dy = frhs(~, y)

dy = zeros(12, 1);

[acceleration,~,~]= env_gravity(0,y(1:3));
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = acceleration;

[acceleration,~,~]= env_gravity(0,y(7:9));
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = acceleration;

end