function config

global truth 
global const
global actuators

%{

Temporary script to initialize global variables until a more robust system is
developed.


%}

%Time
const.INITGPS_WN= 2045;% positive int 
% initial gps week number, epoch for time.

const.mu = 3.986e14;% positive scalar 
% Earth's gravitational constant (m^3/s^2)
const.R_EARTH= 6378137.0;
%Equatorial Radius of Earth (m)*/
const.dt = int64(0.1e9);% positive int64
% Simulation timestep            (ns)
const.e_earth = 0.0167086; 
% Earth's eccentricity.
perihelion_date = datetime(2019,1,3,5,20,0,'TimeZone','UTCLeapSeconds');
const.tp_earth = utl_datetime2time(perihelion_date,const.INITGPS_WN);
% Time when earth was at perihelion (s)
const.period_earth = 365.256363004*24*60*60; 
% Earth orbital period (s)
[rp_earth,vp_earth] = planetEphemeris(juliandate(perihelion_date),'Sun','Earth');
rp_earth = rp_earth';
vp_earth = vp_earth';
h_earth = cross(rp_earth,vp_earth);
const.quat_eci_perifocal = utl_triad([0; 0; 1],[1; 0; 0],h_earth/norm(h_earth),rp_earth/norm(rp_earth));
% Quat between earth's perifocal and eci frame.

const.MAXWHEELRATE= 677.0;% positive scalar
% Max wheel rate in rad/s
const.MAXWHEELRAMP= 304.5;% positive scalar
% Max wheel ramp in rad/s/s
const.MAXMOMENT= 0.08;% positive scalar
% Max magrod moment on one axis (A*m^2)
const.MASS= 4.0;% positive scalar
%dry mass of satellite, kg.
const.JB=[1/12*const.MASS*(0.3^2+0.1^2) 0 0;
          0 1/12*const.MASS*(0.3^2+0.1^2) 0;
          0 0 1/12*const.MASS*(0.1^2+0.1^2);];% 3x3 symmetric matrix
%dry moment of inertia of satellite in body frame
const.JWHEEL=135.0e-7;% positive scalar     
% Wheel Inertia kg*m^2
const.JFUEL_NORM=0.1^2;% positive scalar 
% Moment of inertia of the fuel/mass of the fuel m^2.
const.SLOSH_DAMPING=0.0;% positive scalar
% Torque on fuel/difference in angular rates in eci Nm/(rad/s).
const.ATTITUDE_PD_KP = 75e-4; % imported from simulink
const.ATTITUDE_PD_KD = 32.5e-4; % imported from simulink

% Sensor constants
const.SUNSENSOR_DEADZONE=30*pi/180;% positive scalar
% max angle from +z axis where the sun sensors don't work (rad)

%derived constants
const.JBINV=inv(const.JB);% 3x3 symmetric matrix
% inverse of dry moment of inertia of satellite in body frame




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
truth.angular_rate_body= [0;0;0;];%[10*pi/180;10*pi/180;10*pi/180];
truth.quat_body_eci=[0;0;0;1];
truth.wheel_rate_body=[0;0;0;];
truth.fuel_net_angular_momentum_eci=[0;0;0;];
truth.fuel_mass=0.16;
actuators.wheel_commanded_rate= [0;0;0];
actuators.wheel_commanded_ramp= [0;0;0];
actuators.magrod_real_moment_body= [0;0;0];


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



end
