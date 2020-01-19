function plot_state_transitions(computer_state_trajectory)
%plot_state_transitions plots main_state transitions
N = length(computer_state_trajectory);  % Number of samples
t = 1:N;               % 
main_states=string();
%run through the trajectory and store values
for n=1:N
    main_states(n)=string(computer_state_trajectory{n}.main_state);
end
main_statesc=categorical(main_states);
figure;
scatter(t,main_statesc)

end

