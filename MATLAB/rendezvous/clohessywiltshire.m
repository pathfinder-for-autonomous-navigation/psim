function [phi_rr, phi_rv, phi_vr, phi_vv] = clohessywiltshire(t, n)
% Taken from the best source ever:
% https://en.wikipedia.org/wiki/Clohessy?Wiltshire_equations

phi_rr = [
        4 - 3 * cos(n * t),       0.0, 0.0;
        6 * (sin(n * t) - n * t), 1.0, 0.0;
        0.0,                      0.0, cos(n * t)
];
phi_rv = [
        sin(n * t) / n,               2.0 * (1.0 - cos(n * t)) / n,         0.0;
        2.0 * (cos(n * t) - 1.0) / n, (4.0 * sin(n * t) - 3.0 * n * t) / n, 0.0;
        0.0,                          0.0,                                  sin(n * t) / n
];
phi_vr = [
        3.0 * n * sin(n * t),         0.0, 0.0;
        6.0 * n * (cos(n * t) - 1.0), 0.0, 0.0;
        0.0,                          0.0, -n * sin(n * t)
];
phi_vv = [
        cos(n * t),        2.0 * sin(n * t),       0.0;
        -2.0 * sin(n * t), 4.0 * cos(n * t) - 3.0, 0.0;
        0.0,               0.0,                    cos(n * t)
];

end
