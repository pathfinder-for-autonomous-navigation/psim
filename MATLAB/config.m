function config
%{

Script to initialize const global variables.


%}
global const

setup_path()


%Time
const.INITGPS_WN= 2045;% positive int
% initial gps week number, epoch for time.
const.INIT_DYEAR= decyear(utl_time2datetime(0.0,const.INITGPS_WN));

const.mu = 3986004.415e8;%3.986e14;% positive scalar
% Earth's gravitational constant (m^3/s^2)
const.mu_moon = 4.9048695E12; % positive scalar
% Moon's gravitational constant (m^3/s^2)
const.satArea = 0.1*sqrt(2)*0.3;
%largest planar area of satellite in m^2
const.mu_sun = 1.32712440018E20; % positive scalar
% Sun's gravitational constant (m^3/s^2)
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
const.AU = 149597870700.000000;
% Astronomical unit [m]; DE430
const.c_light   = 299792458.000000000;
% Speed of light  [m/s]; DE430
const.P_Sol = 1367/const.c_light; % [N/m^2] (1367 W/m^2); IERS 96
% Solar radiation pressure at 1 AU
const.Cr = 1; %dimensionless
%Solar radiation pressure coefficient
[rp_earth_moon,vp_earth_moon] = planetEphemeris(juliandate(perihelion_date),'Moon','Earth');
const.rp_earth_moon = 1E3*rp_earth_moon'; %positional vector from Moon to Earth; used for 3rd body perturb calcs
[rp_earth,vp_earth] = planetEphemeris(juliandate(perihelion_date),'Sun','Earth');
const.rp_earth = 1E3*rp_earth; %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
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
const.earth_rate_ecef=[sin(0.2/3600*pi/180);sin(-0.30/3600*pi/180);1;]*7.2921158553E-5;% 3 vector
% earth's inertial rotation rate in ecef frame (rad/s)

const.MAXWHEELRATE= 677.0;% positive scalar
% Max wheel rate in rad/s
const.MAXWHEELRAMP= 304.5;% positive scalar
% Max wheel ramp in rad/s/s
const.MAXMOMENT= 0.08;% positive scalar
% Max magrod moment on one axis (A*m^2)
const.MASS= 4.0;% positive scalar
%dry mass of satellite, kg.
const.JB=[0.03798 0 0;
          0 0.03957 0;
          0 0 0.00688;];% 3x3 symmetric matrix
%Dry moment of inertia of satellite in body frame (kgm^2),
% measurement described here:
% https://cornellprod-my.sharepoint.com/:w:/g/personal/saa243_cornell_edu/EfnqDGLGxSJKsCPZ2Gi0n2UBek152YP_spoqLfRybCa9pQ?e=2S4hV4
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

%% GPS sensor constants %%
const.GPS_LOCK_TIME=0;%1*60;% (positive scalar):
%Time it takes the GPS to get a lock (s)
const.CDGPS_LOCK_TIME=15*60;% (positive scalar):
%Time it takes the CDGPS to get a lock (s)
const.gps_max_angle= pi;%60*pi/180;% (positive scalar):
% Max angle of gps antenna to radia out where gps can work (rad)
const.cdgps_max_angle= 60*pi/180;% (positive scalar):
% Max angle of cdgps antenna to other sat where cdgps can work (rad)
const.cdgps_max_range= 1E3;% (positive scalar):
% Max range of cdgps antenna to other sat where cdgps can work (m)
const.probability_of_ground_gps= 1E-4;% (scalar 0-1):
%Propability of getting a ground gps reading over radio any control cycle the sat
%also can get regular gps.
const.gps_position_bias_sdiv= 1;% (positive scalar):
% standard diviation of bias of gps position measurements (m)
const.cdgps_position_bias_sdiv= 0.05;% (positive scalar):
% standard diviation of bias of cdgps relative position measurements (m)
const.gps_velocity_bias_sdiv= 0.1;% (positive scalar):
% standard diviation of bias of gps velocity measurements (m/s)
const.gps_position_noise_sdiv= 6;% (positive scalar):
% standard diviation of gps position measurements (m)
const.gps_velocity_noise_sdiv= 1;% (positive scalar):
% standard diviation of gps position measurements (m)
%% Magnetometer sensor constants
const.magnetometer_bias_readings_min=50;% (positive int):
%number of readings per axis to get a good magnetometer bias estimate.
const.magnetometer_noise_sdiv= 2E-6;% (positive scalar):
%standard diviation of the magnetometer noise (T)
const.magnetometer_bias_sdiv= 50E-6;% (positive scalar):
%standard diviation of the magnetometer bias (T)
%% Gyro sensor constants
const.gyro_noise_sdiv= 0.1*pi/180;% (positive scalar):
%standard diviation of the gyro noise (rad/s)
const.gyro_bias_sdiv= 1*pi/180;% (positive scalar):
%standard diviation of the gyro bias (rad/s)
%% ORBIT_ESTIMATION parameters %%
const.time_for_stale_cdgps= int64(1E9)*int64(2*60*60);% (int64 scalar):
% time to wait before making the target estimate stale (ns)
sharedposprosdiv= 1E-2;
indposprosdiv= 1E-2;
sharedvelprosdiv= 1E-5;
indvelprosdiv= 1E-6;
selfprocessvar= diag([(indposprosdiv^2+sharedposprosdiv^2)*ones(3,1);(indvelprosdiv^2+sharedvelprosdiv^2)*ones(3,1);]);
coprocessvar= diag([(sharedposprosdiv^2)*ones(3,1);(sharedvelprosdiv^2)*ones(3,1);]);
const.orb_process_noise_var= [selfprocessvar, coprocessvar;
                              coprocessvar',selfprocessvar;];% (12x12 symetric matrix)
% Added variance for bad force models divided by timestep (mks units)
const.orb_self_thrust_noise_sdiv= 0.2;% (positive scalar)
% ratio of thruster impulse that is noise
const.orb_target_thrust_noise_sdiv= 1.0;% (positive scalar)
% ratio of thruster impulse that is noise
gpspossdiv=30;
gpsvelsdiv=10;
cdgpspossdiv=0.5;
const.single_gps_noise_covariance= diag([gpspossdiv^2*ones(3,1);gpsvelsdiv^2*ones(3,1);]);% (6x6 symetric matrix)
%noise covariance of gps reading  (mks units)
const.initial_target_covariance= diag([10;10;10;1E-4;1E-4;1E-4;]);% (6x6 symetric matrix)
%initial covariance used to initialize target state  (mks units)
const.fixed_cdgps_noise_covariance= diag([gpspossdiv^2*ones(3,1);gpsvelsdiv^2*ones(3,1);cdgpspossdiv^2*ones(3,1);]);% (9x9 symetric matrix)
%noise covariance of cdgps reading in fixed mode  (mks units)
const.float_cdgps_noise_covariance= diag([1;1;1;1;1;1;1;1;1;]);% (9x9 symetric matrix)
%noise covariance of cdgps reading in float mode  (mks units)




generate_mex_code();

end
