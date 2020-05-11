clearvars; clc;

% case properties
N_runs = 10;
t_drift = 10 * 24 * 60.0 * 60.0; % Drift time (s)

% generate points on a unit sphere
[x,y,z,N_new] = mySphere(N_runs);

dv_total = zeros(1, N_runs);
t_rendezvous = zeros(1, N_runs);

for ii = 1 : N_runs
    
    % specify velocity vector
    v2 = [x(ii); y(ii); z(ii)];
    
    % run the sim
    two_node_rendezvous_monte_carlo
    
    % record dv and rendezvous time
    dv_total(ii) = sum(dv_norm);
    t_rendezvous(ii) = t(i);
    
end

figure;
plot(dv_total)
xlabel('run')
ylabel('\Delta v [m/s]')
title('Total \Delta v')

figure;
plot(t_rendezvous / 60 / 60 / 24)
xlabel('run')
ylabel('time to rendezvous [days]')
title('time to rendezvous')