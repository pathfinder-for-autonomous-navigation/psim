config()  % Initialize const
generate_mex_code() %generate mex from wrappers


global const
global main_state_trajectory
global computer_state_follower_trajectory
global computer_state_leader_trajectory

dt=double(const.dt) * 1e-9;
t_max = 20000;% Amount of time simulated (s)
t_int = 10.0;% Sampling interval        (s)
num_steps = floor(t_max/dt);
sample_rate = t_int/dt;
N = floor(num_steps /sample_rate);  % Number of samples
main_state= initialize_main_state(1,'detumbled');
[computer_state_follower,computer_state_leader]= initialize_computer_states('not_detumbled');
main_state_trajectory = cell(1,N);
main_state_trajectory{1} = main_state;
computer_state_follower_trajectory = cell(1,N);
computer_state_follower_trajectory{1} = computer_state_follower;
computer_state_leader_trajectory = cell(1,N);
computer_state_leader_trajectory{1} = computer_state_leader;
t_s = 0.0;  % Timestamp of last data point
n   = 1;     % Previously logged data index

for step= 1:num_steps
    sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
    sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);
    %update dynamics
    main_state = main_state_update(main_state);
    %simulate flight computers
    [computer_state_follower,actuator_commands_follower] = update_FC_state(computer_state_follower,sensor_readings_follower);
    [computer_state_leader,actuator_commands_leader] = update_FC_state(computer_state_leader,sensor_readings_leader);
    %command actuators
    main_state.follower = actuator_command(actuator_commands_follower,main_state.follower);
    main_state.leader = actuator_command(actuator_commands_leader,main_state.leader);

    %store trajectory
    if (mod(step,sample_rate)==0)
        n = n + 1;
        t_s = main_state.follower.dynamics.time;
        main_state_trajectory{n} = main_state;
        computer_state_follower_trajectory{n} = computer_state_follower;
        computer_state_leader_trajectory{n} = computer_state_leader;
        fprintf("Progress at %.3f s / %.3f s\n", t_s, t_max);
    end
end
