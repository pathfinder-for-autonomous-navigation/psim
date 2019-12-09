function [X, U] = l1cost(Qn, Q, alpha, A, B, N, xo, rho, eps)
% Source:
%  Finite time, finite horizon, L1 control cost optimization solver.
%  FAST SOLUTION OF OPTIMAL CONTROL PROBLEMS WITH L1 COST
%  Simon Le Cleac?h, Zachary Manchester

% Extract size information
[~, ur] = size(B);

Y = zeros(ur, N - 1);
L = zeros(ur, N - 1);

iter = 1;
max_iter = 5000;

[X, U] = optctrl(Qn, Q, A, B, N, xo, rho, L, Y);
Y = softthresh(U, L, alpha, rho);
L = L + rho .* (U - Y);

while (norm(Y - U) > eps) && (iter < max_iter)
    iter = iter + 1;
    [X, U] = optctrl(Qn, Q, A, B, N, xo, rho, L, Y);
    Y = softthresh(U, L, alpha, rho);
    L = L + rho .* (U - Y);
end

end

function [X, U] = optctrl(Qn, Q, A, B, N, xo, rho, L, Y)

% Extract size information
[xr, ur] = size(B);

% Setup return variables
U = zeros(ur, N - 1);
X = zeros(xr, N);
X(:, 1) = xo;

K = zeros(ur, xr, N - 1);
b = zeros(ur, N - 1);

% Backward pass
g = zeros(xr, 1);
H = Qn;
for i = (N - 1):-1:1
    C = B' * H * A;
    D = rho * eye(ur) + B' * H * B;
    K(:, :, i) = D \ C;
    H = Q + K(:, :, i)' * D * K(:, :, i) - K(:, :, i)' * C - C' * K(:, :, i) + A' * H * A;
    b(:, i) = D \ ( (L(:, i) - rho * Y(:, i)) + B' * g );
    g = A' * g - C' * b(:, i);
end

% Forward pass
for i = 1:(N - 1)
    U(:, i) = - K(:, :, i) * X(:, i) - b(:, i);
    X(:, i + 1) = A * X(:, i) + B * U(:, i);
end
end

function [Y] = softthresh(U, L, alpha, rho)
[r, N] = size(U);
N = N + 1;

Y = zeros(r, N - 1);

tau = alpha / rho;
for i = 1:(N - 1)
    for j = 1:r
        s = U(j, i) + L(j, i) / rho;
        if abs(s) <= tau
            Y(j, i) = 0.0;
        elseif s > tau
            Y(j, i) = s - tau;
        else
            Y(j, i) = s + tau;
        end
    end
end
end
