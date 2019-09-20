function real_actuators = actuator_command(commanded_actuators,state)
%actuator_command Returns the real actuator output from the commands/state
%   given a satelite state and the commanded actuators.
%   real_actuators is a struct with real actuator states.
%       firing_start_times, times since inital GPS week to start firing.
%       real_thrust_vectors_body, real thruster forces, units N.
%       centers_of_thrust_body, center of thrust for each firing, units m.
%       firing_on_times, how long firings last.
%       wheel_commanded_rate, commanded x,y,z wheel rate.
%       wheel_commanded_ramp, commanded x,y,z wheel ramp, units rad/s/s.
%       magrod_real_moment_body, real magnetorquer moment, units A*m^2
%   commanded_actuators is a struct with actuator commands:
%       firing_start_times, commanded time since inital GPS week to start firing.
%       commanded_impulse_vectors_eci, commanded impulse, units N.
%       wheel_commanded_rate, commanded x,y,z wheel rate.
%       wheel_commanded_ramp, commanded x,y,z wheel ramp, units rad/s/s.
%       magrod_moment, magnetorquer commanded moment, units A*m^2
%   state are a structs with elements:
%       time, time since inital GPS week.
%       position_eci, position of the center of mass of the satellite.
%       velocity_eci, velocity of the center of mass of the satellite.
%       angular_rate_body, the angular rate of the spacecraft in the body frame.
%       quat_body_eci, quaternion that rotates from eci to body frame.
%       wheel_rate_body, x,y, and z, wheel angular rates.
%       fuel_net_angular_momentum_eci, net angular momentum of the fuel.
%       fuel_mass, the mass of the fuel.
global const
real_actuators.magrod_real_moment_body=min(max(commanded_actuators.magrod_moment,-const.MAXMOMENT),const.MAXMOMENT)+const.RESIDUAL_MOMENT;
real_actuators.wheel_commanded_rate=commanded_actuators.wheel_commanded_rate;
real_actuators.wheel_commanded_ramp=commanded_actuators.wheel_commanded_ramp;
%TODO add thruster commands, and descritized ramp for wheels
end

