config();
main_state = initialize_main_state(1,'default');
actuator_commands = struct();
main_state.follower=actuator_command(actuator_commands,main_state.follower);
assert(isstruct(main_state.follower),'main_state.follower is not a struct')
