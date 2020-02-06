function actuators = actuators_off_state()
%actuators_off_state Constructs an `actuators` that is the state with all of the actuators off. 

% actuators is a struct with elements:
%    * `thrust_vectors_body`(3x4 matrix): each thruster's force vector (N)
%    * `centers_of_thrust_body`(3x4 matrix): Center of thrust of each truster (m)
%    * `wheel_commanded_rate`(3x1 matrix): Commanded x,y,z wheel rate (rad/s)
%    * `wheel_commanded_ramp`(3x1 matrix): Commanded x,y,z wheel ramp (rad/s/s)
%    * `magrod_real_moment_body`(3x1 matrix): Real magnetorquer moment (Am^2)
%    * `magrod_hysteresis_body`(3x1 matrix): Real magnetorquer hysteresis moment (Am^2)
%    * `ground_position_ecef`(3x1 matrix): ground known estimated position of the satellite (m)
%    * `ground_velocity_ecef`(3x1 matrix): ground known estimated velocity of the gps reciever of the satellite (m/s)
%    * `ground_time`(scalar): ground known estimated time since initial GPS week (s)
%    * `current_thruster_force_body` (3x1 matrix): Net thruster force vector at current timestep (N)
%    * `current_thruster_torque_body` (3x1 matrix): Net thruster torque vector at current timestep (N-m)
actuators=struct();
actuators.thrust_vectors_body= zeros(3,4);%TODO fill out with real max thrust vectors.
actuators.centers_of_thrust_body= zeros(3,4);%TODO fill out with real thruster positions.
actuators.current_thruster_force_body= zeros(3,1);
actuators.current_thruster_torque_body= zeros(3,1);
actuators.wheel_commanded_rate= zeros(3,1);
actuators.wheel_commanded_ramp= inf(3,1);
actuators.magrod_real_moment_body= zeros(3,1);
actuators.magrod_hysteresis_body= zeros(3,1);
actuators.ground_position_ecef= nan(3,1);
actuators.ground_velocity_ecef= nan(3,1);
actuators.ground_time= nan;
end

