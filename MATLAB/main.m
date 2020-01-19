config()  % Initialize const

global const
global main_state_trajectory
global computer_state_follower_trajectory
global computer_state_leader_trajectory
global rng_state_trajectory

dt=double(const.dt) * 1e-9;
t_max = 20000;% Amount of time simulated (s)
t_int = 10.0;% Sampling interval        (s)
num_steps = floor(t_max/dt);
sample_rate = t_int/dt;
condition='detumbled';%'tumbling';%
main_state= initialize_main_state(1,condition);
[computer_state_follower,computer_state_leader]= initialize_computer_states(condition);
[main_state_trajectory, ...
    computer_state_follower_trajectory, ...
    computer_state_leader_trajectory, ...
    rng_state_trajectory] = run_sim(num_steps, ...
    sample_rate, ...
    main_state, ...
    computer_state_follower, ...
    computer_state_leader);