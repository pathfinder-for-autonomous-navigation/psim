function config

global truth const

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

const.mu = 3.986e14;      % Earth's gravitational constant (m^3/s^2)
const.dt = int64(0.1e9);  % Simulation timestep            (ns)

truth.mission_time = int64(0);  % Mission time (ns)
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
