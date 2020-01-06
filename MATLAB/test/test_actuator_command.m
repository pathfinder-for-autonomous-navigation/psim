main_state = initialize_main_state(1,'default');
actuator_commands = struct();
actuator_commands.firing_start_times= [];
actuator_commands.commanded_impulse_vectors_eci= [];
actuator_commands.wheel_torque= zeros(3,1);
actuator_commands.wheel_enable= zeros(3,1);
actuator_commands.magrod_moment= zeros(3,1);
main_state.follower=actuator_command(actuator_commands,main_state.follower);
assert(isstruct(main_state.follower),'main_state.follower is not a struct')
