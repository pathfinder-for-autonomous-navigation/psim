function [X, U] = lqr(Qn, Q, R, A, B, N, xo)
% Finite time, finite horizon LQR solver with unique final stage cost.

% Extract size information
[xr, ur] = size(B);

% Setup return variables
U = zeros(ur, N - 1);
X = zeros(xr, N);
X(:, 1) = xo;

F = zeros(ur, xr, N - 1);
P = Qn;

for i = (N - 1):-1:1
    F(:, :, i) = (R + B' * P * B) \ (B' * P * A);
    P = A' * P * A - (A' * P * B) * ((R + B' * P * B) \ (B' * P * A)) + Q;
end

for i = 1:(N - 1)
    U(:, i) = - F(:, :, i) * X(:, i);
    X(:, i + 1) = A * X(:, i) + B * U(:, i);
end
end

