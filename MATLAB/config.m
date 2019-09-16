function config

global truth 
global const
global actuators

%{

Temporary script to initialize global variables until a more robust system is
developed.

Global variables treated as inputs:

Global variables treated as outputs:
 * const.mu
 * const.dt
 * truth.mission_time
 * truth.a
 * truth.e
 * truth.i
 * truth.O
 * truth.o
 * truth.nu
 * truth.r
 * truth.v

%}

const.mu = 3.986e14;% positive scalar 
% Earth's gravitational constant (m^3/s^2)
const.dt = int64(0.1e9);% positive int64
% Simulation timestep            (ns)
const.INITGPS_WN= 2045;% positive int 
% initial gps week number, epoch for time.
const.MAXWHEELRATE= 677.0;% positive scalar
% Max wheel rate in rad/s
const.MAXWHEELRAMP= 304.5;% positive scalar
% Max wheel ramp in rad/s/s
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

%derived constants
const.JBINV=inv(const.JB);% 3x3 symmetric matrix
% inverse of dry moment of inertia of satellite in body frame


truth.mission_time = int64(0);% int64
% Mission time (ns)
% ^^ Should always initially be zero

truth.a  = 6860636.6;  % Semimajor axis                        (m)
truth.e  = 0.001;      % Eccentricity                          (unitless)
truth.i  = 45*pi/180;  % Inclination angle                     (rad)
truth.O  = 0.0;        % Right ascension of the ascending node (rad)
truth.o  = 0.0;        % Argument of perigee                   (rad)
truth.nu = 0*pi/180;   % True anamoly                          (rad)

[   truth.r,...  % Position (m)   [eci]
    truth.v,...  % Velocity (m/s) [eci]
] = utl_orb2rv(truth.a*(1-truth.e), truth.e, truth.i, truth.O, truth.o, truth.nu, const.mu);

truth.time= double(truth.mission_time)*1E-9;
truth.position_eci= truth.r;
truth.velocity_eci= truth.v;
truth.angular_rate_body= [10*pi/180;10*pi/180;10*pi/180];
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

% % Julian date of the GPS epoch
% const.init_epoch__jd = juliandate('6-Jan-1980', 'dd-mmm-yyyy');
% % Mission start date in nanoseconds sense the epoch
% const.init_gps_time__ns = int64( 9.158e17 );  % About epoch + 40 yrs
% % Leap seconds from the epoch till current mission time
% const.leap_time__ns = int64( 0 );
% truth.m__kg = 3.7;  % Initial mass
% % Timestep of the simulation (not the integration timestep as ode45 is used)
