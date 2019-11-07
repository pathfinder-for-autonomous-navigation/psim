function satellite_state = actuator_command(actuator_commands,satellite_state)
%actuator_command Returns the real actuator output from the commands/state
%   given a satelite state and the commanded actuators.
global const

%replace NaNs with zeros
actuator_commands.magrod_moment(isnan(actuator_commands.magrod_moment)) = 0;
actuator_commands.wheel_enable(isnan(actuator_commands.wheel_enable)) = 0;
actuator_commands.wheel_torque(isnan(actuator_commands.wheel_torque)) = 0;
actuator_commands.wheel_torque(isnan(actuator_commands.wheel_torque)) = 0;
actuator_commands.wheel_torque(isnan(actuator_commands.wheel_torque)) = 0;

satellite_state.actuators.magrod_real_moment_body=min(max(actuator_commands.magrod_moment,-const.MAXMOMENT),const.MAXMOMENT);
satellite_state.actuators.wheel_commanded_rate=actuator_commands.wheel_enable.*sign(actuator_commands.wheel_torque)*const.MAXWHEELRATE;
satellite_state.actuators.wheel_commanded_ramp=abs(actuator_commands.wheel_torque)/const.JWHEEL;
%TODO add thruster commands, and descretized ramp for wheels

%send estimated orbit to ground
% try to send ground reading
if (satellite_state.sensors.gps_time_till_lock<=0 && rand()<const.probability_of_ground_gps)
    satellite_state.actuators.ground_position_ecef= actuator_commands.position_ecef;
    satellite_state.actuators.ground_velocity_ecef= actuator_commands.velocity_ecef;
    satellite_state.actuators.ground_time= actuator_commands.time;
end

t = satellite_state.dynamics.time; % get current time
%t = t*ones(size(satellite_state.actuators.firing_on_times));

dt = double(const.dt) * 1e-9; % simulation dt

% proportion of current timestep that thrusters are on for
time_on = [0 0 0 0]';
percent_on = [0 0 0 0]';
for i = 1:4
    % compute proportion (to see this graphically, visit https://www.desmos.com/calculator/dcl7jii0us)
    time_on(i) = min([dt,... % timestep length (if thruster firing entirely envelopes dt)
        (actuator_commands.firing_start_times(i) + actuator_commands.firing_on_times(i) - t),... % time between timestep start and thruster off
        t + dt - actuator_commands.firing_start_times(i),... % time between thruster on and timestep end
        actuator_commands.firing_on_times(i)]); % length of thruster firing (if thruster firing is completely within timestep)

    time_on(i) = max(time_on(i),0); % set to 0 if firing is completely outside of timestep)

    percent_on(i) = time_on(i)/dt; % convert to percent of timestep
end
%current_thrusters_on = (t > satellite_state.actuators.firing_start_times)&(t < (satellite_state.actuators.firing_start_times + satellite_state.actuators.firing_on_times)); % 4x1 matrix, whether each thruster should be on

current_thrust_vectors_body = satellite_state.actuators.thrust_vectors_body; % initialize current thrust vectors matrix
for i = 1:4
    %if current_thrusters_on(i)==0 % if thruster is off
    %    current_thrust_vectors_body(:,i) = zeros(3,1); % set thruster vector to 0
    %end
    current_thrust_vectors_body(:,i) = current_thrust_vectors_body(:,i)*percent_on(i); % multiply thrust vector by percentage of time on
    current_thrust_vectors_body(:,i) = current_thrust_vectors_body(:,i).*((2*rand(3,1)-1)*0.01*0+1); % +/-1% noise
end
satellite_state.actuators.current_thruster_force_body = sum(current_thrust_vectors_body,2); % total thrust vector is sum of individual thrust vectors

current_thrust_torques_body = zeros(3,4); % initialize thruster torques matrix
for i = 1:4
    % Calculate cross product
    current_thrust_torques_body(:,i) =  cross(satellite_state.actuators.centers_of_thrust_body(:,i),satellite_state.actuators.thrust_vectors_body(:,i)); % compute torque from each thruster
end
satellite_state.actuators.current_thruster_torque_body = sum(current_thrust_torques_body,2); % sum all torque

% figure out what proportion of the timestep each thruster is on for to
% calculate the "average force" over the computer time step of 0.1 seconds,
% make sure to subtract the initial time before the thruster turns on and
% time after the thruster turns off, and saturate it if the thruster is on
% the whole time
end
