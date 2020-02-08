function plot_gps_lock(main_state_trajectory)
%plot_gps_lock plots the time till gps and cdgps lock
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
%run through the trajectory and store values
for n=1:N
    f_gps_lock(n)= max(main_state_trajectory{n}.follower.sensors.gps_time_till_lock,0);
    l_gps_lock(n)= max(main_state_trajectory{n}.leader.sensors.gps_time_till_lock,0);
    cdgps_lock(n)= max(main_state_trajectory{n}.leader.sensors.cdgps_time_till_lock,0);
    t(n) = main_state_trajectory{n}.follower.dynamics.time;  % Take initial data
end

figure;
plot(t,[f_gps_lock; l_gps_lock;cdgps_lock;])
legend('follower time to GPS lock','leader time to GPS lock','time to CDGPS lock')
title('Time to GPS lock')
xlabel('time (s)')
ylabel('time to lock (s)')
