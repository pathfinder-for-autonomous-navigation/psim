function plot_state_transitions(computer_state_trajectory,main_state_trajectory)
%plot_state_transitions plots main_state transitions
N = length(computer_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data    % 
main_states=string();
%run through the trajectory and store values
for n=1:N
    main_states(n)=string(computer_state_trajectory{n}.main_state);
    t(n) = main_state_trajectory{n}.follower.dynamics.time;  % Take initial data
end
main_statesc=categorical(main_states);
figure;
scatter(t,main_statesc)
title('computer main state')
xlabel('time (s)')
ylabel('state name')

end

