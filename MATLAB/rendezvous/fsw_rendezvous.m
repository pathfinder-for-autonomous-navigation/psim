
% Satellite properaties
M = 5.0;               % Mass (kg)
J_min = 2e-4;          % Min impulse (Ns)
J_max = 2e-2;          % Max impulse (Ns)
dt_fire = 5.0 * 60.0;  % Minimum time between firings (s)

% Deployment properties
V_rel   = 0.6;               % Relative velocity at deployment (m/s)
t_drift = 5.0 * 90.0 * 60.0; % Drift time (s)

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.0;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...
    v1,...
] = utl_orb2rv(a * (1 - e), e, i, O, o, nu, 3.986e14);
n = sqrt(3.986e14 / (a * a * a));  % Orbital rate (rad/s)
w_hill = [0.0; 0.0; n];            % Angular rate of the hill frame

% Rendezvous algorithm properties
N = 18;          % Number of nodes (gives horizon of ~1 orbit)
eps = 1e-4;      % Tolerance value
rho = 1.0;       % Penalty weigth
alpha = 1.0;     % L1 cost factor
[rr, rv, vr, vv] = clohessywiltshire(dt_fire, n); % State update matrices
A = [rr, rv; vr, vv];
B = [rv; vv] ./ M;
Qn = 1000.0 .* eye(6);  % Final state cost
Q  =    0.0 .* eye(6);  % Intermediate state cost

% Simulation properties
sim_tol = 1.0;       % Error bound in position and velocity to stop the sim
sim_max_iter = 2500; % Maximum number of sim iterations

% Add initial velocity difference and perform the drift phase
r2 = r1;
v2 = randn(3, 1);
v2 = v1 + V_rel * v2 / norm(v2);
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift, 0);

X = zeros(6, sim_max_iter);
U = zeros(3, sim_max_iter);
% x_err = zeros(1, sim_max_iter);
dv_tot = zeros(1, sim_max_iter);

% Calculate our initial state
Q_eci_hill = eci_to_hill(r1, v1);
r_hill = Q_eci_hill * (r2 - r1);
v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);

X(:, 1) = [r_hill; v_hill];
U(:, 1) = [0.0; 0.0; 0.0];
% x_err(1) = norm(r_hill) + norm(v_hill);
dv_tot(1) = 0.0;
iter = 1;

opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-2, 'InitialStep', 0.1);
while norm(r_hill) + norm(v_hill) > sim_tol && iter < sim_max_iter
    iter = iter + 1

    % Determine the next actuation
    [~, u] = l1cost(Qn, Q, alpha, A, B, N, [r_hill; v_hill], rho, eps);
    u(:, 1) = (abs(u(:, 1)) > J_min) .* u(:, 1);
    u(:, 1) = max(u(:, 1), -J_max);
    u(:, 1) = min(u(:, 1),  J_max);

    % Apply the actuation
    v2 = v2 + Q_eci_hill' * u(:, 1) ./ M;

    % Simulate the dynamics
    [~, y] = ode45(@frhs, [0.0, dt_fire], [r1; v1; r2; v2], opt);
    r1 = y(end, 1:3)';
    v1 = y(end, 4:6)';
    r2 = y(end, 7:9)';
    v2 = y(end, 10:12)';
    
    % Recalculate hill frame information
    Q_eci_hill = eci_to_hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    
    X(:, iter) = [r_hill; v_hill];
    U(:, iter) = u(:, 1);
    % x_err(iter) = norm(r_hill) + norm(v_hill);
    dv_tot(iter) = dv_tot(iter - 1) + norm(u(:, 1)) / M;
    
end

figure
hold on
plot(X(1, 1:iter), '-r')
plot(X(2, 1:iter), '-g')
plot(X(3, 1:iter), '-b')
hold off
title('Relative Position of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Position (m)')

figure
hold on
plot(X(4, 1:iter), '-r')
plot(X(5, 1:iter), '-g')
plot(X(6, 1:iter), '-b')
hold off
title('Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

figure
hold on
plot(U(1, 1:iter), '-r')
plot(U(2, 1:iter), '-g')
plot(U(3, 1:iter), '-b')
hold off
title('Actuation of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Impulse (Ns)')

% figure
% plot(x_err(1:iter), '-b')
% title('State Error (rgb ~ xyz LVLH)')
% xlabel('Time (s)')
% ylabel('Error')

figure
plot(dv_tot(1:iter), '-b')
title('Total Delta V Used')
xlabel('Time (s)')
ylabel('Delta V (m/s)')


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


function Q = eci_to_hill(r, v)

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
