clearvars; clc;

global const
global truth_trajectory
global computer_state_trajectory
global actuators_trajectory

addpath('utl');
addpath('environmental_models');
addpath('environmental_models/helper_functions');

dt=double(const.dt) * 1e-9;
t_max = 2000;%0.01 * 90.0 * 60.0;  % Amount of time simulated (s)
t_int = 10.0;               % Sampling interval        (s)
%arrays to plot
num_steps = floor(t_max/dt);
sample_rate = t_int/dt;
N = floor(num_steps /sample_rate);  % Number of samples
config()  % Initialize the simulation

[truth,actuators,sensor_state,computer_state]=initialize_states(1,'not_detumbled');
computer_state_trajectory = cell(1,N);
sensor_state_trajectory{1} = sensor_state;
actuators_trajectory = cell(1,N);
actuators_trajectory{1} = actuators;
truth_trajectory = cell(1,N);
truth_trajectory{1} = truth;
t_s = 0.0;  % Timestamp of last data point
n   = 1;     % Previously logged data index

for step= 1:num_steps
    %sense truth
    tic;
    sensor_readings = sensor_reading(sensor_state,truth,actuators);
    t_sensors = toc;
    %update dynamics
    tic;
    truth = orbit_attitude_update_ode2(truth,actuators,dt);
    t_orbit_att = toc;
    %update time
    truth.mission_time = truth.mission_time+const.dt;
    truth.time = double(truth.mission_time)*1E-9;
    %update sensor state (for example biases)
    tic;
    sensor_state_update(sensor_state,truth,dt);
    t_sensor_state = toc;
    %simulate flight computer
    tic;
    [computer_state,actuator_commands] = update_FC_state(computer_state,sensor_readings);
    t_fc = toc;
    %command actuators
    tic;
    actuators = actuator_command(actuator_commands,truth);
    t_actuators = toc;
    
    if (mod(step,sample_rate)==0)
        n = n + 1;
        t_s = double(truth.mission_time) * 1e-9;
        truth_trajectory{n} = truth;
        actuators_trajectory{n} = actuators;
        computer_state_trajectory{n} = computer_state;
        tumbling_momentum=norm(const.JB*truth.angular_rate_body)
        fprintf("Progress at %.3f s / %.3f s\n", t_s, t_max);
    end
end

% check timings
t_sensors
t_orbit_att
t_sensor_state
t_fc
t_actuators

plot_trajectory(truth_trajectory);