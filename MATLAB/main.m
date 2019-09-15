clearvars; clc;

global truth
global const

addpath('utl');
addpath('environmental_models');
addpath('environmental_models/helper_functions');

t_max = 1000;%0.01 * 90.0 * 60.0;  % Amount of time simulated (s)
t_int = 10.0;               % Sampling interval        (s)

%arrays to plot
N = floor(t_max / t_int) + 1;  % Number of samples
t = zeros(1, N);               % Stored mission time data
r = zeros(3, N);               % Stored position
orbital_energys= zeros(1, N); 
orbital_angular_momentum_ecis= zeros(3, N);
rotational_energys= zeros(1, N);
spacecraft_angular_momentum_ecis= zeros(3, N);

config()  % Initialize the simulation

[orbital_energys(:,1),orbital_angular_momentum_ecis(:,1),rotational_energys(:,1),spacecraft_angular_momentum_ecis(:,1)]=almost_conserved_values(truth);
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
        [orbital_energys(:,n),orbital_angular_momentum_ecis(:,n),rotational_energys(:,n),spacecraft_angular_momentum_ecis(:,n)]=almost_conserved_values(truth);
        fprintf("Progress at %.3f s / %.3f s\n", t_s, t_max);
    end
end

% Plot a projection of the the trajectory onto the ECI xy-plane
figure;
plot(r(1,:),r(2,:))
title("orbit")
figure;
plot(t,orbital_energys-orbital_energys(1))
title("delta orbital_energys")
figure;
plot(t,(orbital_angular_momentum_ecis-orbital_angular_momentum_ecis(:,1))')
title("delta orbital_angular_momentum_ecis")
figure;
plot(t,rotational_energys-rotational_energys(1))
title("delta rotational_energys")
figure;
plot(t,(spacecraft_angular_momentum_ecis-spacecraft_angular_momentum_ecis(:,1))')
title("delta spacecraft_angular_momentum_eci")
