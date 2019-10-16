function main_state = main_state_update(main_state)
%main_state_update Updates the main state over one delta time step.
main_state.leader.dynamics= dynamics_update(main_state.leader.dynamics,main_state.leader.actuators);
main_state.follower.dynamics= dynamics_update(main_state.follower.dynamics,main_state.follower.actuators);

main_state.follower.sensors= sensors_update(main_state.follower.sensors,main_state.follower.dynamics);
main_state.leader.sensors= sensors_update(main_state.leader.sensors,main_state.leader.dynamics);


end

