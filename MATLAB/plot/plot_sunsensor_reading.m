function plot_sunsensor_reading(main_state_trajectory)
%plot_sunsensor_reading plots the sun sensor reading.
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
sunsensor_reading_f=zeros(3, N);
sunsensor_reading_l=zeros(3, N);

%run through the trajectory and store values
for n=1:N
    main_state= main_state_trajectory{n};
    sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);
    sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
    sunsensor_reading_f(:,n)=sensor_readings_follower.sat2sun_body;
    sunsensor_reading_l(:,n)=sensor_readings_leader.sat2sun_body;
    t(n) = main_state.leader.dynamics.time;  % Take initial data
end


figure;
plot(t,sunsensor_reading_f)
title('sunsensor reading follower')
xlabel('time (s)')
ylabel('sun sensor reading')
legend('x','y','z')
figure;
plot(t,sunsensor_reading_l)
title('sunsensor reading leader')
xlabel('time (s)')
ylabel('sun sensor reading')
legend('x','y','z')
