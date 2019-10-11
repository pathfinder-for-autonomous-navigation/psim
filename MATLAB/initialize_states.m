function [truth,actuators,sensor_state,computer_state] = initialize_states(seed,condition)
%initialize_states Sample an initial state. 
%   To set the seed based on current time, use seed = 'shuffle'
%
%   initial_state and final state are a structs with elements:
%       time, datetime time zone UTC leepseconds.
%       position_eci, position of the center of mass of the satellite.
%       velocity_eci, velocity of the center of mass of the satellite.
%       angular_rate_body, the angular rate of the spacecraft in the body frame.
%       quat_body_eci, quaternion that rotates from eci to body frame.
%       wheel_rate_body, x,y, and z, wheel angular rates.
%       fuel_net_angular_momentum_eci, net angular momentum of the fuel.
%       fuel_mass, the mass of the fuel.
%   actuators is a struct with actuator inputs that are constant over the
%   following time step but not constant for the whole simulation:
%       firing_start_times, times since inital GPS week to start firing.
%       real_thrust_vectors_body, real thruster forces, units N.
%       centers_of_thrust_body, center of thrust for each firing, units m.
%       firing_on_times, how long firings last.
%       wheel_commanded_rate, commanded x,y,z wheel rate.
%       wheel_commanded_ramp, commanded x,y,z wheel ramp, units rad/s/s.
%       magrod_real_moment_body, real magnetorquer moment, units A*m^2

rng(seed,'threefry')
global const
truth=struct();
actuators=struct();
sensor_state=struct();
computer_state=struct();

computer_state.adcs_state='init';
switch condition
    case 'detumbled'
        truth.angular_rate_body= [0;0;0;];
    otherwise
        truth.angular_rate_body=randn(3,1)*5*pi/180;
end
a  = 6860636.6;  % Semimajor axis                        (m)
e  = 0.001;      % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)

[   r,...  % Position (m)   [eci]
    v,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a*(1-e), e, i, O, o, nu, const.mu);

truth.mission_time = int64(0);% int64
% Mission time (ns)
truth.time= double(truth.mission_time)*1E-9;
truth.position_eci= r;
truth.velocity_eci= v;
truth.quat_body_eci=[0;0;0;1];
truth.wheel_rate_body=[0;0;0;];
truth.fuel_net_angular_momentum_eci=[0;0;0;];
truth.fuel_mass=0.16;
actuators.wheel_commanded_rate= [0;0;0];
actuators.wheel_commanded_ramp= [0;0;0];
actuators.magrod_real_moment_body= [0;0;0];
end

