function [final_state,actuators] = update_FC_state(initial_state,sensor_readings)
% Global variables treated as inputs:
%  * const
%  * sensors
% 
% Global variables treated as outputs:
%  * actuators
%  * 

% attitude PD controller
%   calculate commanded wheel torque
%   update state with commanded wheel torque

global const

quat_cmd = utl_array2quaternion([0;0;1;0;]);

%% Attitude determination

sat2sun_eci= env_sun_vector(sensor_readings.time);
mag_ecef= env_magnetic_field(sensor_readings.time,sensor_readings.position_ecef);
[quat_ecef_eci,~]=env_earth_attitude(sensor_readings.time);
quat_ecef_eci= utl_array2quaternion(quat_ecef_eci);
quat_eci_ecef= conj(quat_ecef_eci);
mag_eci =rotateframe(quat_eci_ecef,mag_ecef')';
quat_body_eci=utl_triad(sat2sun_eci,mag_eci,sensor_readings.sat2sun_body,sensor_readings.magnetometer_body);
quat_body_eci = utl_array2quaternion(quat_body_eci);

%% PD controller 

quat_error =  conj(quat_cmd) * quat_body_eci;
if parts(quat_error) < 0 %make sure the forth componet is positive
    quat_error = -quat_error;
end

q = utl_quaternion2array(quat_error);
actuators.wheel_torque = q(1:3)*const.ATTITUDE_PD_KP + sensor_readings.gyro_body*const.ATTITUDE_PD_KD;
actuators.magrod_moment= [0;0;0;];
actuators.wheel_enable=[1;1;1;];
final_state=initial_state;

end