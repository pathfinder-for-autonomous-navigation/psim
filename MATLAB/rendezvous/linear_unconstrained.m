t = 5 * 60;   % Time in seconds between each actuation
N = 20;       % Number of actuations plus one
M = 5.0;      % Mass of the chaser spacecraft in kg
n = 0.00113;  % Orbital rate of the leader
              % From https://en.wikipedia.org/wiki/Clohessy?Wiltshire_equations
eps   = 1e-3; % Tolerance value (value from Standford paper was 1e-5 but has convergence issues)
rho   = 1.0;  % Penalty weigth (value from Stanford paper)
alpha = 1.0;  % L1 cost parameter (value from Stanford paper)

xo = [8.0; 10.0;  0.1; -2.2;  0.16; 0.0];  % Initial conditions

[rr, rv, vr, vv] = clohessywiltshire(t, n);
A = [rr, rv;
     vr, vv];
B = [rv;
     vv] ./ M;

Qn = 1000.0 .* eye(6);  % Final state cost
Q  =    0.0 .* eye(6);  % Intermediate state cost

[X, U] = l1cost(Qn, Q, alpha, A, B, N, xo, rho, eps);

figure
hold on
plot(X(1, :), '-r')
plot(X(2, :), '-g')
plot(X(3, :), '-b')
hold off
title('Position (m) (rbg ~ xyz)')

figure
hold on
plot(X(4, :), '-r')
plot(X(5, :), '-g')
plot(X(6, :), '-b')
hold off
title('Velocity (m/s) (rbg ~ xyz)')

figure
hold on
plot(U(1, :), '-r')
plot(U(2, :), '-g')
plot(U(3, :), '-b')
hold off
title('L1 Cost Actuation (Ns) (rgb ~ xyz)')
