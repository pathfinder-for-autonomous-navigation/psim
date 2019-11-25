function actuator_commands = actuators_off_command()
%actuator_off_command Constructs an `actuator_commands` that commands all of the actuators off. 

% actuator_commands is a struct with elements:
% firing_start_times(4x1 matrix): commanded times since initial GPS week to start firing (s)
% firing_on_times(4x1 matrix): commanded thruster on times (s)
% wheel_torque, commanded x,y,z wheel torque, (signed ramp)x(rotor inertia) (Nm)
% wheel_enable, commanded x,y,z wheel enables, whether each wheel should be on, if false, the wheel rate is commanded to zero.
% magrod_moment, commanded x,y,z magnetorquer moments (Am^2)
actuator_commands= struct();
actuator_commands.firing_start_times= inf(4,1);
actuator_commands.firing_on_times= zeros(4,1);
actuator_commands.wheel_torque= inf(3,1);
actuator_commands.wheel_enable= false(3,1);
actuator_commands.magrod_moment= zeros(3,1);
end

