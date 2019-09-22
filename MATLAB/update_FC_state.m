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
quat_cmd = [0;0;1;0;];%utl_array2quaternion(initial_state.quat_cmd);
quat_body_eci = utl_array2quaternion(sensors.quat_body_eci);
quat_error =  conj(quat_cmd) * quat_body_eci;
if parts(quat_error) < 0
    quat_error = -quat_error;
end

q = utl_quaternion2array(quat_error);
actuators.wheel_torque = q(1:3)*const.ATTITUDE_PD_KP + sensors.angular_rate_body*const.ATTITUDE_PD_KD;
actuators.magrod_moment= [0;0;0;];
final_state=initial_state;

end