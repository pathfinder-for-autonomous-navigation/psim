function [X, U, iter] = ren_l1cost(Qn, Q, alpha, A, B, N, x0, rho, epsilon, max_iter)
% Solves a finite time, finite horizon, optimal control problem with L1 control
% cost. See utl_l1costiter for more information.

[~, c] = size(B);
Y = zeros(c, N - 1);
L = zeros(c, N - 1);

iter = 1;
[X, U, Y, L, delta] = ren_l1costiter(Qn, Q, alpha, A, B, N, x0, rho, Y, L);
while (delta >= epsilon) && (iter <= max_iter)
    [X, U, Y, L, delta] = ren_l1costiter(Qn, Q, alpha, A, B, N, x0, rho, Y, L);
    iter = iter + 1;
end
end
