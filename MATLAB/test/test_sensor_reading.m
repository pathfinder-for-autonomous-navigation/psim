main_state = initialize_main_state(1,'default');
sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);
assert(isstruct(sensor_readings_follower),'sensor_readings_follower is not a struct')
assert(isstruct(sensor_readings_leader),'sensor_readings_leader is not a struct')