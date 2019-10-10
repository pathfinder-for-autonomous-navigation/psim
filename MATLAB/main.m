clearvars; clc;

global const
global truth_trajectory

addpath('utl');
addpath('environmental_models');
addpath('environmental_models/helper_functions');

t_max = 10;%0.01 * 90.0 * 60.0;  % Amount of time simulated (s)
t_int = 1.0;               % Sampling interval        (s)
%arrays to plot
N = floor(t_max / t_int) + 1;  % Number of samples
config()  % Initialize the simulation

[truth,actuators,sensor_state,computer_state]=initialize_states('shuffle','detumbled');
truth_trajectory = cell(1,N);
truth_trajectory{1} = truth;
t_s = 0.0;  % Timestamp of last data point
n   = 1;     % Previously logged data index

while truth.mission_time * 1e-9 < t_max
    
    %sense truth
    tic;
    sensor_readings = sensor_reading(sensor_state,truth,actuators);
    t_sensors = toc;
    %update dynamics
    tic;
    truth = orbit_attitude_update_ode2(truth,actuators,double(const.dt) * 1e-9);
    t_orbit_att = toc;
    %update time
    truth.mission_time = truth.mission_time+const.dt;
    truth.time = double(truth.mission_time)*1E-9;
    %update sensor state (for example biases)
    tic;
    sensor_state_update(sensor_state,truth,double(const.dt) * 1e-9);
    t_sensor_state = toc;
    %simulate flight computer
    tic;
    [computer_state,actuator_commands] = update_FC_state(computer_state,sensor_readings);
    t_fc = toc;
    %command actuators
    tic;
    actuators = actuator_command(actuator_commands,truth);
    t_actuators = toc;
    
    if t_s + t_int <= truth.mission_time * 1e-9
        n = n + 1;
        t_s = truth.mission_time * 1e-9;
        truth_trajectory{n} = truth;
        fprintf("Progress at %.3f s / %.3f s\n", t_s, t_max);
    end
end

% check timings
t_sensors
t_orbit_att
t_sensor_state
t_fc
t_actuators

% plot_trajectory(truth_trajectory);