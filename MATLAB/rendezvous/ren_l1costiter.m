function [X, U, Y, L, delta] = ren_l1costiter(Qn, Q, alpha, A, B, N, x0, rho, Y, L)
% Steps a single iteration of a finite time, finite horizon optimal control
% problem with L1 control cost, quadratic stage cost, and quadratic final
% stage cost.
%
%   i.e. min_U  x' Qn x + sum_i x_i' Q x_i + alpha |u_i|
%     where x_i+1 = A x_i + B u_i
%     and X contains N total states
%
% Source:
%  Finite time, finite horizon, L1 control cost optimization solver.
%  FAST SOLUTION OF OPTIMAL CONTROL PROBLEMS WITH L1 COST
%  Simon Le Cleac?h, Zachary Manchester

[X, U] = optimal_control_update(Qn, Q, A, B, N, x0, rho, L, Y);
Y = soft_threshold_update(U, L, alpha, rho);
L = L + rho .* (U - Y);
delta = norm(Y - U);

end


function [X, U] = optimal_control_update(Qn, Q, A, B, N, x0, rho, L, Y)

[r, c] = size(B);

K = zeros(c, r, N - 1);
b = zeros(c, N - 1);
U = zeros(c, N - 1);
X = zeros(r, N);
X(:, 1) = x0;

% Backward pass
g = zeros(r, 1);
H = Qn;
for i = (N - 1):-1:1
    C = B' * H * A;
    D = rho * eye(c) + B' * H * B;
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


function Y = soft_threshold_update(U, L, alpha, rho)

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
