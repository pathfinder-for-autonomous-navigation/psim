function config
%{

Script to initialize const global variables.


%}
global const

[filepath, name, ext] = fileparts(mfilename("fullpath"));
addpath(strcat(filepath, '/utl'));
addpath(strcat(filepath, '/environmental_models'));
addpath(strcat(filepath, '/environmental_models/helper_functions'));
addpath(strcat(filepath, '/plot'));
addpath(strcat(filepath, '/adcs'));


%Time
const.INITGPS_WN= 2045;% positive int 
% initial gps week number, epoch for time.
const.INIT_DYEAR= decyear(utl_time2datetime(0.0,const.INITGPS_WN));

const.mu = 3986004.415e8;%3.986e14;% positive scalar 
% Earth's gravitational constant (m^3/s^2)
const.R_EARTH= 6378137.0;
%Equatorial Radius of Earth (m)*/
const.dt = int64(1e8);% positive int64
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
T0=utl_time2datetime(0,const.INITGPS_WN);%pan epoch
T5=T0+years(5);%5 years after pan epoch
dcm_ECEF0_ECI=dcmeci2ecef('IAU-2000/2006',[year(T0),month(T0),day(T0),hour(T0),minute(T0),second(T0)]);
dcm_ECEF5_ECI=dcmeci2ecef('IAU-2000/2006',[year(T5),month(T5),day(T5),hour(T5),minute(T5),second(T5)]);
polarprecessionaxis= -cross((dcm_ECEF0_ECI*dcm_ECEF5_ECI'*[0;0;1;]),[0;0;1;]);
const.PRECESSION_RATE= polarprecessionaxis/seconds(T5-T0);% 3 vector
% earth's axis precession rate (rad/s)
const.quat_ecef0_eci= utl_quaternion2array(quaternion(dcm_ECEF0_ECI,'rotmat','frame'));
%ecef0 is ecef frame at time 0 inertialy stuck.
const.earth_rate_ecef=[0;0;7.2921158553E-5;];% 3 vector
% earth's inertial rotation rate in ecef frame (rad/s)

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
%dry moment of inertia of the satellite in body frame (kg*m^2)
const.JWHEEL=135.0e-7;% positive scalar     
% Wheel Inertia kg*m^2
const.JFUEL_NORM=0.1^2;% positive scalar 
% Moment of inertia of the fuel/mass of the fuel m^2.
const.SLOSH_DAMPING=0.0;% positive scalar
% Torque on fuel/difference in angular rates in eci Nm/(rad/s).
const.ATTITUDE_PD_KP = 20.0e-4; % imported from simulink
const.ATTITUDE_PD_KD = 22.5e-4; % imported from simulink
const.detumble_safety_factor= 0.2; % (scalar range (0,1)):
% The fraction of max wheel momentum detumbling ends at.

%derived constants
const.JBINV=inv(const.JB);% 3x3 symmetric matrix
% inverse of dry moment of inertia of satellite in body frame

const.GPS_LOCK_TIME=15*60;% positive scalar
%Time it takes the GPS to get a lock (s)
const.CDGPS_LOCK_TIME=15*60;% positive scalar
%Time it takes the CDGPS to get a lock (s)

const.magnetometer_bias_readings_min=50;% (positive int): 
%number of readings per axis to get a good magnetometer bias estimate.
const.magnetometer_noise_sdiv= 2E-6;% (positive scalar):
%standard diviation of the magnetometer noise (T)
const.magnetometer_bias_sdiv= 50E-6;% (positive scalar):
%standard diviation of the magnetometer bias (T)
const.gyro_noise_sdiv= 0.1*pi/180;% (positive scalar):
%standard diviation of the gyro noise (rad/s)
const.gyro_bias_sdiv= 1*pi/180;% (positive scalar):
%standard diviation of the gyro bias (rad/s)




generate_mex_code();

end
