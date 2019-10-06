function [final_state,actuators] = update_FC_state(initial_state,sensors)
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
sat2sun_eci= env_sun_vector(sensors.time);
mag_ecef= env_magnetic_field(sensors.time,sensors.position_ecef);
[quat_ecef_eci,~]=env_earth_attitude(sensors.time);
quat_ecef_eci= utl_array2quaternion(quat_ecef_eci);
quat_eci_ecef= conj(quat_ecef_eci);
mag_eci =rotateframe(quat_eci_ecef,mag_ecef')';
quat_body_eci=utl_triad(sat2sun_eci,mag_eci,sensors.sat2sun_body,sensors.magnetometer_body);
quat_body_eci = utl_array2quaternion(quat_body_eci);
%% PD controller 
quat_error =  conj(quat_cmd) * quat_body_eci;
if parts(quat_error) < 0 %make sure the forth componet is positive
    quat_error = -quat_error;
end

q = utl_quaternion2array(quat_error);
actuators.wheel_torque = q(1:3)*const.ATTITUDE_PD_KP + sensors.gyro_body*const.ATTITUDE_PD_KD;
actuators.magrod_moment= [0;0;0;];
actuators.wheel_enable=[1;1;1;];
final_state=initial_state;

end