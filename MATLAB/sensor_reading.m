function [sensor_readings] = sensor_reading(my_satellite_state,other_satellite_state)
%sensor_reading returns the sensor readings
%   TODO implement the actual sensors with errors
%   TODO implement GPS and CDGPS
%   TODO use get_truth function.

global const
sensor_readings= struct();
true_state=my_satellite_state.dynamics;


%% quaternions

[quat_ecef_eci,~]=env_earth_attitude(true_state.time);
quat_ecef_eci= utl_array2quaternion(quat_ecef_eci);
quat_body_eci= utl_array2quaternion(true_state.quat_body_eci);
quat_eci_ecef= conj(quat_ecef_eci);
quat_body_ecef=quat_eci_ecef*quat_body_eci;

%% gyro reading

sensor_readings.gyro_body= true_state.angular_rate_body;

%% magnetometer reading

position_ecef=rotateframe(quat_ecef_eci,true_state.position_eci')';
B_ecef= env_magnetic_field(true_state.time,position_ecef);
sensor_readings.magnetometer_body=rotateframe(quat_body_ecef,B_ecef')';

%% sun sensor reading

sat2sun_eci=env_sun_vector(true_state.time);
sat2sun_body=rotateframe(quat_body_eci,sat2sun_eci')';

%now determine if the satellite is in eclipse with earth.
eclipse = env_eclipse(true_state.position_eci,sat2sun_eci);
sun_in_dead_zone=(sat2sun_body(3))<-cos(30*pi/180);

%we can't read the sun sensor because the sun is in the dead zone
if ((~eclipse) && (~sun_in_dead_zone))
    sensor_readings.sat2sun_body=rotateframe(quat_body_eci,sat2sun_eci')';
    sensor_readings.sun_sensor_true= true;
else
    sensor_readings.sat2sun_body= NaN(3,1);
    sensor_readings.sun_sensor_true= false;
end

%% wheel angular momentum reading

sensor_readings.wheel_momentum_body= true_state.wheel_rate_body*const.JWHEEL;

%% GPS

sensor_readings.time= true_state.time;
sensor_readings.position_ecef= rotateframe(quat_ecef_eci,true_state.position_eci')';
%TODO get velocity in ECEF


end

