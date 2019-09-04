clearvars; clc;

global truth

addpath('utl');

t_max = 5.0 * 90.0 * 60.0;  % Amount of time simulated (s)
t_int = 60.0;               % Sampling interval        (s)

N = floor(t_max / t_int) + 1;  % Number of samples
t = zeros(1, N);               % Stored mission time data
r = zeros(3, N);               % Stored position

config()  % Initialize the simulation

t(:, 1) = truth.mission_time * 1e-9;  % Take initial data
r(:, 1) = truth.r;

t_s = t(1);  % Timestamp of last data point
n   = 1;     % Previously logged data index

while truth.mission_time * 1e-9 < t_max
    
    truth_update()  % Update the simulation
    
    if t_s + t_int <= truth.mission_time * 1e-9
        n = n + 1;
        t(:, n) = truth.mission_time * 1e-9;
        r(:, n) = truth.r;
        t_s = t(n);
        fprintf("Progress at %.3f s / %.3f s\n", t_s, t_max);
    end
end

% Plot a projection of the the trajectory onto the ECI xy-plane
figure;
plot(r(1,:),r(2,:))
