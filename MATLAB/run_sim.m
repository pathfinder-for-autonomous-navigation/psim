function [main_state_trajectory,computer_state_follower_trajectory,computer_state_leader_trajectory,rng_state_trajectory] = run_sim(num_steps,sample_rate,main_state,computer_state_follower,computer_state_leader)
%run_sim Runs a simulation, outputting the trajectory sampled every sample_rate
global const
N = floor(num_steps /sample_rate);  % Number of samples
dt=double(const.dt) * 1e-9;
main_state_trajectory = cell(1,N);
main_state_trajectory{1} = main_state;
computer_state_follower_trajectory = cell(1,N);
computer_state_follower_trajectory{1} = computer_state_follower;
computer_state_leader_trajectory = cell(1,N);
computer_state_leader_trajectory{1} = computer_state_leader;
rng_state_trajectory= cell(1,N);
rng_state_trajectory{1}= rng;
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
        rng_state_trajectory{n}= rng;
        main_state_trajectory{n} = main_state;
        computer_state_follower_trajectory{n} = computer_state_follower;
        computer_state_leader_trajectory{n} = computer_state_leader;
        fprintf("Progress at %.3f s / %.3f s\n", step*dt, num_steps*dt);
    end
end
end