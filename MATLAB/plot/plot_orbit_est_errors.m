function plot_orbit_est_errors(computer_state_trajectory,main_state_trajectory)
%plot_orbit_est_errors plots main_state transitions
N = length(computer_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data    % 
error_r_ecef=zeros(3, N);
error_v_ecef=zeros(3, N);
error_rel_r_ecef=zeros(3, N);
error_rel_v_ecef=zeros(3, N);
%run through the trajectory and store values
for n=1:N
    dynamics_f= main_state_trajectory{n}.follower.dynamics;
    dynamics_l= main_state_trajectory{n}.leader.dynamics;
    if (strcmp(computer_state_trajectory{n}.type,'leader'))
        dynamics= dynamics_l;
        other_dynamics= dynamics_f;
    else
        dynamics= dynamics_f;
        other_dynamics= dynamics_l;
    end
    
    orbit_est_state=computer_state_trajectory{n}.orbit_est_state;
    t(n) = dynamics.time;  % Take initial data
    %update orbit_est_state to the right time
    [~,self2target_r_ecef,self2target_v_ecef,r_ecef,v_ecef] ...
    = orb_run_estimator(...
    orbit_est_state,false,false,false,nan(3,1),nan(3,1),nan(3,1),dynamics.time_ns,nan(3,1),nan(3,1),0);

    error_r_ecef(:,n)= r_ecef-get_truth('position ecef',dynamics);
    error_v_ecef(:,n)= v_ecef-get_truth('velocity ecef',dynamics);
    error_rel_r_ecef(:,n)= self2target_r_ecef-(get_truth('position ecef',other_dynamics)-get_truth('position ecef',dynamics));
    error_rel_v_ecef(:,n)= self2target_v_ecef-(get_truth('velocity ecef',other_dynamics)-get_truth('velocity ecef',dynamics));
end
figure;
plot(t,vecnorm(error_r_ecef))
title('error self position ecef')
xlabel('time (s)')
ylabel('Distance (m)')

figure;
plot(t,vecnorm(error_v_ecef))
title('error self velocity ecef')
xlabel('time (s)')
ylabel('Speed (m/s)')

figure;
plot(t,vecnorm(error_rel_r_ecef))
title('error target relative position ecef')
xlabel('time (s)')
ylabel('Distance (m)')

figure;
plot(t,vecnorm(error_rel_v_ecef))
title('error target relative velocity ecef')
xlabel('time (s)')
ylabel('Speed (m/s)')

end

