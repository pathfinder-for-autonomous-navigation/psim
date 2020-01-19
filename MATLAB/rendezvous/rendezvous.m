
M = 5.0; % Mass (kg)
J_min = 2e-4; % Min impulse (Ns)
J_max = 2e2;%2e-2; % Max impulse (Ns)
V_rel   = 0.5; % Relative velocity at deployment (m/s)
t_drift = 5.0 * 90.0 * 60.0; % Drift time (s)
dt_fire = 60.0; % Minimum time between firings (s)
N = 110; % Gives a horizon a little over an orbit
% dx_tol = 1e-1; % x' * x must be less than this to stop the simulation
% max_iter = floor(2.0 * 30.0 * 24.0 * 3600.0 / dt_fire); % About two months

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.0;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e), e, i, O, o, nu, 3.986e14);

n = sqrt(3.986e14 / (a * a * a));     % Orbital rate
w_hill = [0.0; 0.0; n];

% Add initial velocity difference
r2 = r1;
v2 = randn(3, 1);
v2 = v1 + V_rel * v2 / norm(v2);

% Allow spacecraft to drift apart
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift, 0);

eps   = 1e-4; % Tolerance value (value from Standford paper was 1e-5 but has convergence issues)
rho   = 1.0;  % Penalty weigth (value from Stanford paper)
alpha = 1.0;  % L1 cost factor (value from Stanford paper)
[rr, rv, vr, vv] = clohessywiltshire(dt_fire, n); % State update matrices
A = [rr, rv; vr, vv];
B = [rv; vv] ./ M;
Qn = 1000.0 .* eye(6);  % Final state cost
Q  =    0.0 .* eye(6);  % Intermediate state cost

Q_transform = eci_to_lvlh(r1, v1);
r_hill = Q_transform * (r2 - r1);
v_hill = Q_transform * (v2 - v1) - cross(w_hill, r_hill);
[X, U] = l1cost(Qn, Q, alpha, A, B, N, [r_hill; v_hill], rho, eps);

figure
hold on
plot(X(1, :), '-r')
plot(X(2, :), '-g')
plot(X(3, :), '-b')
hold off
title('Planned Relative Position of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Position (m)')

figure
hold on
plot(X(4, :), '-r')
plot(X(5, :), '-g')
plot(X(6, :), '-b')
hold off
title('Planned Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

figure
hold on
plot(U(1, :), '-r')
plot(U(2, :), '-g')
plot(U(3, :), '-b')
hold off
title('Actuation of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Impulse (Ns)')


Q_transform = eci_to_lvlh(r1, v1);
r_hill = Q_transform * (r2 - r1);
v_hill = Q_transform * (v2 - v1) - cross(w_hill, r_hill);
X(:, 1) = [r_hill; v_hill];
opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-2, 'InitialStep', 0.1);
for i = 1:(N - 1)
    
    % Apply actuation
    v2 = v2 + Q_transform' * U(:, i) / M;
    
    % Simulate dynamics
    [~, y] = ode45(@frhs, [0.0, dt_fire], [r1; v1; r2; v2], opt);
    r1 = y(end, 1:3)';
    v1 = y(end, 4:6)';
    r2 = y(end, 7:9)';
    v2 = y(end, 10:12)';
    
    % Calculate new state
    Q_transform = eci_to_lvlh(r1, v1);
    r_hill = Q_transform * (r2 - r1);
    v_hill = Q_transform * (v2 - v1) - cross(w_hill, r_hill);
    X(:, i + 1) = [r_hill; v_hill];
    
end

figure
hold on
plot(X(1, :), '-r')
plot(X(2, :), '-g')
plot(X(3, :), '-b')
hold off
title('Resulting Relative Position of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Position (m)')

figure
hold on
plot(X(4, :), '-r')
plot(X(5, :), '-g')
plot(X(6, :), '-b')
hold off
title('Resulting Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')


function [t, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift, plot_flag)
% Performs the drift phase of the simulation. Propegates the satellites
% orbits to 't_drift' seconds and plots the position and velocity of
% satellite two relative to satellite one if plot is true.

opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-2, 'InitialStep', 0.1);
[t, y] = ode45(@frhs, [0.0, t_drift], [r1; v1; r2; v2], opt);

if plot_flag
    
    dy = zeros(length(t), 6);
    for i = 1:length(t)
        Q = eci_to_lvlh(y(i, 1:3)', y(i, 4:6)');
        dy(i, 1:3) = (y(i, 7:9) - y(i, 1:3)) * Q';
        dy(i, 4:6) = (y(i, 10:12) - y(i, 4:6)) * Q';
    end
    
    figure
    hold on
    plot(t, dy(:, 1), '-r')
    plot(t, dy(:, 2), '-g')
    plot(t, dy(:, 3), '-b')
    hold off
    title('Drift Phase Relative Position of Satellite Two (rgb ~ xyz LVLH)')
    xlabel('Time (s)')
    ylabel('Position (m)')
    
end

t = t(end);
r1 = y(end, 1:3)';
v1 = y(end, 4:6)';
r2 = y(end, 7:9)';
v2 = y(end, 10:12)';

end


function Q = eci_to_lvlh(r, v)

r_hat = -r / norm(r);
n_hat = cross(r, v) / norm(cross(r, v));
v_hat = cross(n_hat, r_hat);

Q = [r_hat'; v_hat'; n_hat'];

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
