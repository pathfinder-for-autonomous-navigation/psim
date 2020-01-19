main_state = initialize_main_state(1,'default');
new_dynamcics = dynamics_update(main_state.follower.dynamics,main_state.follower.actuators);
assert(isstruct(main_state),'new_dynamcics is not a struct')