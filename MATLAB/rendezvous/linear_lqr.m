t = 240;      % Time in seconds between each actuation
N = 90;       % Number of actuations plus one
M = 5.0;      % Mass of the chaser spacecraft in kg
n = 0.00113;  % Orbital rate of the leader
              % From https://en.wikipedia.org/wiki/Clohessy?Wiltshire_equations

xo = [2.0; 0.3;  10.0; -2.2;  0.16; 0.2];  % Initial conditions

[rr, rv, vr, vv] = clohessywiltshire(t, n);
A = [rr, rv;
     vr, vv];
B = [rv;
     vv] ./ M;

Qn = 1000.0 .* eye(6);  % Final state cost
Q  =    0.0 .* eye(6);  % Intermediate state cost
R  =    0.5 .* eye(3);  % Control cost

[X, U] = lqr(Qn, Q, R, A, B, N, xo);

figure
hold on
plot(X(1, :), '-r')
plot(X(2, :), '-g')
plot(X(3, :), '-b')
hold off
title('Position with Actuation (m) (rbg ~ xyz)')

figure
hold on
plot(X(4, :), '-r')
plot(X(5, :), '-g')
plot(X(6, :), '-b')
hold off
title('Velocity with Actuation (m) (rbg ~ xyz)')

figure
hold on
plot(U(1, :), '-r')
plot(U(2, :), '-g')
plot(U(3, :), '-b')
hold off
title('Actuation (Ns) (rgb ~ xyz)')
