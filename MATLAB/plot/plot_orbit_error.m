function plot_orbit_error(main_state_trajectory)
%plot_orbit_error plots differences in leader and follower orbits.
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
relative_pos_eci=zeros(3, N);
relative_vel_eci=zeros(3, N);
%run through the trajectory and store values
for n=1:N
    dynamics_f= main_state_trajectory{n}.follower.dynamics;
    dynamics_l= main_state_trajectory{n}.leader.dynamics;
    relative_pos_eci(:,n)= get_truth('position eci',dynamics_l)-get_truth('position eci',dynamics_f);
    relative_vel_eci(:,n)= get_truth('velocity eci',dynamics_l)-get_truth('velocity eci',dynamics_f);

    t(n) = dynamics_f.time;  % Take initial data
end


figure;
plot(t,vecnorm(relative_pos_eci))
title('Distance (m)')
xlabel('time (s)')
ylabel('Distance (m)')

figure;
plot(t,vecnorm(relative_vel_eci))
title('Relative Velocity Magnitude (m/s)')
xlabel('time (s)')
ylabel('Relative Velocity Magnitude (m/s)')
