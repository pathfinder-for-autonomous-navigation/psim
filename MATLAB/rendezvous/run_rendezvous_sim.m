clearvars; clc;

% case properties
N_runs = 10;
t_days = 1 : 1 : 10;

% generate points on a unit sphere
[xs, ys, zs, N_new] = mySphere(N_runs);

dv_total = zeros(length(t_days), N_new);
t_rendezvous = zeros(length(t_days), N_new);

for iii = 1 : length(t_days)
    
    t_drift = t_days(iii) * 24 * 60.0 * 60.0; % Drift time (s)

    for ii = 1 : N_new

        % specify velocity vector
        v2 = [xs(ii); ys(ii); zs(ii)];

        % run the sim
        two_node_rendezvous_monte_carlo

        % record dv and rendezvous time
        dv_total(iii, ii) = sum(dv_norm);
        t_rendezvous(iii, ii) = t(i);

    end
    
end

%%

figure;
plot(dv_total', 'o')
xlabel('run')
ylabel('\Delta v [m/s]')
title('Total \Delta v')
grid on
% legend('1 day', '2 days', '3 days', '4 days', '5 days', '6 days', '7 days', '8 days', '9 days', '10 days')

figure;
plot(t_rendezvous' / 60 / 60 / 24, 'o')
xlabel('run')
ylabel('time to rendezvous [days]')
title('time to rendezvous')
grid on

r_init = r_init / norm(r_init) * 50;
v_init = v_init / norm(v_init) * 50;

figure;
quiver3(0, 0, 0, r_init(1), r_init(2), r_init(3)); hold on
quiver3(0, 0, 0, v_init(1), v_init(2), v_init(3));

for i = 1 : N_new
    
    time_vec = t_rendezvous(end, i) * [xs(i); ys(i); zs(i)] / 60 / 60 / 24;
    quiver3(0, 0, 0, time_vec(1), time_vec(2), time_vec(3), 'k');
    
end

legend('r_1', 'v_1');
title('time to rendezvous for each vector')
% axis equal

r_init = r_init / norm(r_init) * 5;
v_init = v_init / norm(v_init) * 5;

figure;
quiver3(0, 0, 0, r_init(1), r_init(2), r_init(3)); hold on
quiver3(0, 0, 0, v_init(1), v_init(2), v_init(3));

for i = 1 : N_new
    
    total_dv_vec = dv_total(end, i) * [xs(i); ys(i); zs(i)];
    quiver3(0, 0, 0, total_dv_vec(1), total_dv_vec(2), total_dv_vec(3), 'k');
    
end

legend('r_1', 'v_1');
title('dv for each vector')
% axis equal

%%

subplot(1, 2, 1)
for i = 1 : N_new
    scatter(t_days, dv_total(:, i), 'k', 'filled'); hold on
end
xlabel('drift time [days]')
ylabel('total \Delta v [m/s]')

subplot(1, 2, 2)
for i = 1 : N_new
    scatter(t_days, t_rendezvous(:, i), 'k', 'filled'); hold on
end
xlabel('drift time [days]')
ylabel('time to rendezvous [days]')