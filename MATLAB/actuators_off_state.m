function actuators = actuators_off_state()
%actuators_off_state Constructs an `actuators` that is the state with all of the actuators off. 

% actuators is a struct with elements:
% firing_start_times, Times since initial GPS week to start firing each thruster (s)
% thrust_vectors_body, each thruster's force vector (N)
% centers_of_thrust_body, Center of thrust of each truster (m)
% firing_on_times, How long each thruster firing lasts (s)
% wheel_commanded_rate, Commanded x,y,z wheel rate (rad/s)
% wheel_commanded_ramp, Commanded x,y,z wheel ramp (rad/s/s)
% magrod_real_moment_body, Real magnetorquer moment (Am^2)
% magrod_hysteresis_body, Real magnetorquer hysteresis moment (Am^2)
actuators=struct();
actuators.firing_start_times= inf(4,1);
actuators.thrust_vectors_body= zeros(3,4);
actuators.centers_of_thrust_body= zeros(3,4);
actuators.firing_on_times= zeros(4,1);
actuators.wheel_commanded_rate= zeros(3,1);
actuators.wheel_commanded_ramp= inf(3,1);
actuators.magrod_real_moment_body= zeros(3,1);
actuators.magrod_hysteresis_body= zeros(3,1);
end

