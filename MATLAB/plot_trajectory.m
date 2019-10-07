function plot_trajectory(truth_trajectory)
%plot_trajectory makes plots of the truth trajectory
%   Detailed explanation goes here
% Plot a projection of the the trajectory onto the ECI xy-plane
global const
N = length(truth_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
r = zeros(3, N);               % Stored position
wheel_speed = zeros(3, N);               % Stored wheel speed
q = zeros(4, N);               % Stored quat_body_eci
orbital_energys= zeros(1, N); 
orbital_angular_momentum_ecis= zeros(3, N);
rotational_energys= zeros(1, N);
spacecraft_angular_momentum_ecis= zeros(3, N);
%run through the trajectory and store values
for n=1:N
    t(:, n) = truth_trajectory{n}.time;  % Take initial data
    r(:, n) = truth_trajectory{n}.position_eci;
    q(:, n) = truth_trajectory{n}.quat_body_eci;
    wheel_speed(:, n) = truth_trajectory{n}.wheel_rate_body;
    [orbital_energys(:,n),orbital_angular_momentum_ecis(:,n),rotational_energys(:,n),spacecraft_angular_momentum_ecis(:,n)]=almost_conserved_values(truth_trajectory{n});
end

figure;
plot(t,q(1,:),t,q(2,:),t,q(3,:),t,q(4,:))
title("quaternion_body_eci")
xlabel('time (s)')
ylabel('quaternion component')

figure;
plot(t,wheel_speed(1,:),t,wheel_speed(2,:),t,wheel_speed(3,:))
title("wheel speed")
xlabel('time (s)')
ylabel('wheel speed (rad/s)')

figure;
plot(r(1,:),r(2,:))
title("orbit")
xlabel('x position (m)')
ylabel('Y position (m)')

figure;
plot(t,orbital_energys-orbital_energys(1))
title("delta orbital energy")
xlabel('time (s)')
ylabel('energy (J)')

figure;
plot(t,(orbital_angular_momentum_ecis-orbital_angular_momentum_ecis(:,1))')
title("delta orbital angular momentum eci")
xlabel('time (s)')
ylabel('angular momentum (Nms)')

figure;
plot(t,rotational_energys-rotational_energys(1))
title("delta rotational energy")
xlabel('time (s)')
ylabel('energy (J)')

figure;
plot(t,(spacecraft_angular_momentum_ecis-spacecraft_angular_momentum_ecis(:,1))')
title("delta spacecraft angular momentum eci")
xlabel('time (s)')
ylabel('angular momentum (Nms)')
end

