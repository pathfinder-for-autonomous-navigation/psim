function config

global truth 
global const

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
const.SLOSH_DAMPING=0.1;% positive scalar
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





end

% % Julian date of the GPS epoch
% const.init_epoch__jd = juliandate('6-Jan-1980', 'dd-mmm-yyyy');
% % Mission start date in nanoseconds sense the epoch
% const.init_gps_time__ns = int64( 9.158e17 );  % About epoch + 40 yrs
% % Leap seconds from the epoch till current mission time
% const.leap_time__ns = int64( 0 );
% truth.m__kg = 3.7;  % Initial mass
% % Timestep of the simulation (not the integration timestep as ode45 is used)
