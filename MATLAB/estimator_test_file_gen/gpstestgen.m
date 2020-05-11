% A script to generate sensors and truth for testing the 
% gps orbit estimator

sensors= struct();
truth= struct();

config()  % Initialize const
global const

dt=double(const.dt) * 1e-9;
t_max = 24*60*60;% Amount of time simulated (s)
num_steps = floor(t_max/dt);
condition='detumbled';%'tumbling';%
seed= 1;
main_state= initialize_main_state(seed,condition);
[computer_state_follower,computer_state_leader]= initialize_computer_states(condition);

for step= 1:num_steps
    sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
    %sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);
    sensors(step).t= sensor_readings_follower.gpstime;
    sensors(step).r= sensor_readings_follower.position_ecef;
    sensors(step).v= sensor_readings_follower.velocity_ecef;
    truth(step).t= int64(main_state.follower.dynamics.time_ns)+const.INIT_GPSNS;
    truth(step).r= get_truth('position ecef',main_state.follower.dynamics);
    truth(step).v= get_truth('velocity ecef',main_state.follower.dynamics);
    %update dynamics
    main_state = main_state_update(main_state);
    %simulate flight computers
    [computer_state_follower,actuator_commands_follower] = update_FC_state(computer_state_follower,sensor_readings_follower);
    %[computer_state_leader,actuator_commands_leader] = update_FC_state(computer_state_leader,sensor_readings_leader);
    %command actuators
    main_state.follower = actuator_command(actuator_commands_follower,main_state.follower);
    %main_state.leader = actuator_command(actuator_commands_leader,main_state.leader);
end

%% Save file

[filepath,~,~] = fileparts(mfilename('fullpath'));
filename = fullfile(filepath, "../../estimatortest/test-files/gps-from-matlab-sim.hdf5");
sensors_docs= "GPS sensor data in ECEF and GPS time (ns, m, m/s)";
truth_docs= "MATLAB sim orbit data in ECEF and GPS time (ns, m, m/s)";

hdf5_helper(filename,sensors,sensors_docs,truth,truth_docs,string(mfilename),seed,condition);