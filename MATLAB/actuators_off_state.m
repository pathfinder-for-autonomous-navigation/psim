function actuators = actuators_off_state()
%actuators_off_state Constructs an `actuators` that is the state with all of the actuators off. 

% actuators is a struct with elements:
%    * `firing_start_times`(4x1 matrix): Times since initial GPS week to start firing each thruster (s)
%    * `thrust_vectors_body`(3x4 matrix): each thruster's force vector (N)
%    * `centers_of_thrust_body`(3x4 matrix): Center of thrust of each truster (m)
%    * `firing_on_times`(4x1 matrix): How long each thruster firing lasts (s)
%    * `wheel_commanded_rate`(3x1 matrix): Commanded x,y,z wheel rate (rad/s)
%    * `wheel_commanded_ramp`(3x1 matrix): Commanded x,y,z wheel ramp (rad/s/s)
%    * `magrod_real_moment_body`(3x1 matrix): Real magnetorquer moment (Am^2)
%    * `magrod_hysteresis_body`(3x1 matrix): Real magnetorquer hysteresis moment (Am^2)
%    * `ground_position_ecef`(3x1 matrix): ground known estimated position of the satellite (m)
%    * `ground_velocity_ecef`(3x1 matrix): ground known estimated velocity of the gps reciever of the satellite (m/s)
%    * `ground_time`(scalar): ground known estimated time since initial GPS week (s)
actuators=struct();
actuators.firing_start_times= inf(4,1);
actuators.thrust_vectors_body= zeros(3,4);
actuators.centers_of_thrust_body= zeros(3,4);
actuators.firing_on_times= zeros(4,1);
actuators.wheel_commanded_rate= zeros(3,1);
actuators.wheel_commanded_ramp= inf(3,1);
actuators.magrod_real_moment_body= zeros(3,1);
actuators.magrod_hysteresis_body= zeros(3,1);
actuators.ground_position_ecef= nan(3,1);
actuators.ground_velocity_ecef= nan(3,1);
actuators.ground_time= nan;
end

