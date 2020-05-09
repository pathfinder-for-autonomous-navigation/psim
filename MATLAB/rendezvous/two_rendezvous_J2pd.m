
M = 4.0; % Mass (kg)
max_dv_thrust= 2e-2 / M;
V_rel   = 1.0; % Relative velocity at deployment (m/s)
t_drift = 20.0 * 90.0 * 60.0; % Drift time (s)
    
mu = 398600.440e-9;

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.001;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e*e), e, i, O, o, nu, mu);

n = utl_orbrate(a);      % Orbital rate
w_hill = [0.0; 0.0; n];

% Add initial velocity difference
r2 = r1;
v2 = randn(3, 1);
v2 = v1 + V_rel * (v2 / norm(v2));

% Allow spacecraft to drift apart
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift);

N= 1000;
dvs = zeros(N, 1);
dps = zeros(N, 1);
des = zeros(N, 1);
dns = zeros(N, 1);

for i = 1:(N - 1)

    r1_m = r1;%+0.1*randn(3,1);
    v1_m = v1;%+0.001*randn(3,1);
    r2_m = r2;%+0.1*randn(3,1);
    v2_m = v2;%+0.001*randn(3,1);

    %just use energy pd
    [dv, ~, ~, ~] = controller(r1_m, v1_m, r2_m, v2_m);

    % Simulate dynamics
    dt_fire= 3*pi/2/n;
    [~, r1, v1, r2, v2] = drift_phase(r1, v1 + dv, r2, v2, dt_fire);
    r1_m = r1;%+0.1*randn(3,1);
    r2_m = r2;%+0.1*randn(3,1);
    v1_m = v1;%+0.001*randn(3,1);
    v2_m = v2;%+0.001*randn(3,1);

    %just use energy pd
    [dv, dp, de, dn] = controller(r1_m, v1_m, r2_m, v2_m);
    
    dvs(i) = norm(dv);
    dps(i) = dp;
    des(i) = de;
    dns(i) = dn;
    
    % Simulate dynamics
    dt_fire= pi/2/n;
    [~, r1, v1, r2, v2] = drift_phase(r1, v1 + dv, r2, v2, dt_fire);

end

figure
plot(dvs)
title('dv')

figure
plot(dps)
title('dp')

figure
plot(de)
title('de')

figure
plot(dn)
title('dn')

% figure
% hold on
% plot(X(1, :), '-r')
% plot(X(2, :), '-g')
% plot(X(3, :), '-b')
% hold off
% title('Resulting Relative Position of Satellite Two (rgb ~ xyz LVLH)')
% xlabel('Time (s)')
% ylabel('Position (m)')

% figure
% hold on
% plot(X(4, :), '-r')
% plot(X(5, :), '-g')
% plot(X(6, :), '-b')
% hold off
% title('Resulting Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
% xlabel('Time (s)')
% ylabel('Velocity (m/s)')

% figure;
% plot(deltaenergies);


function [dv, dp, de, dn] = controller(r1, v1, r2, v2)
    dv = zeros(3, 1);

    % Calculate our hill state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    
    % Calculate phase
    dp = atan2(norm(r1) + r_hill(1), r_hill(2)) % outputs [-pi, pi]

    % Calculat energies
    U1 = grav_pot(r1);
    U2 = grav_pot(r2);
    de = (0.5 * dot(v2, v2) - U2) - (0.5 * dot(v1, v1) - U1)
    
    % Phase, energy PD
    dvp_mag = 1.0 * dp + 1.0 * de;
    dv = dv + dvp_mag * r1 ./ norm(r1);
    
    % Angular momentum controller
    h1 = cross(r1, v1);
    h2 = cross(r2, v2);
    dh = h2 - h1 * dot(h1, h2) ./ norm(h1, 'fro');
    dn = norm(dh);
    dvn_mag = 1.0 * dot(cross(h1./norm(h1), r1./norm(r1)), dh)
    dv = dv + dvn_mag * h1./norm(h1);

end

function [t, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift)
% Performs the drift phase of the simulation. Propegates the satellites
% orbits to 't_drift' seconds and plots the position and velocity of
% satellite two relative to satellite one if plot is true.

opt = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
[t, y] = ode45(@frhs, [0.0, t_drift], [r1; v1; r2; v2], opt);

t = t(end);
r1 = y(end, 1:3)';
v1 = y(end, 4:6)';
r2 = y(end, 7:9)';
v2 = y(end, 10:12)';

end

function dy = frhs(~, y)

dy = zeros(12, 1);

[gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = [gx; gy; gz];

[gx, gy, gz] = gravityzonal(y(7:9)', 'Earth', 4, 'Error');
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = [gx; gy; gz];

end

function U = grav_pot(r)
mu = 3.98600440e14;
    j = 0;%1.75553e-5; % https://en.wikipedia.org/wiki/Geopotential_model
    U = - mu / norm(r) + j / norm(r, 'fro') * (r(3) / norm(r));
end
