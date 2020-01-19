main_state = initialize_main_state(1,'default');
new_sensors = sensors_update(main_state.follower.sensors,main_state.follower.dynamics);
assert(isstruct(main_state),'new_sensors is not a struct')