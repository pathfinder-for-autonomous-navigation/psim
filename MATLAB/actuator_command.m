function satellite_state = actuator_command(actuator_commands,satellite_state)
%actuator_command Returns the real actuator output from the commands/state
%   given a satelite state and the commanded actuators.
global const

%replace NaNs with zeros
actuator_commands.magrod_moment(isnan(actuator_commands.magrod_moment)) = 0;
actuator_commands.wheel_enable(isnan(actuator_commands.wheel_enable)) = 0;
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
end

