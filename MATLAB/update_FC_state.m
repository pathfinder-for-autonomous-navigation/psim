function [final_state,actuators] = update_FC_state(initial_state,const,sensors)
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
quat_cmd = utl_array2quaternion(initial_state.quat_cmd);
quat_body_eci = utl_array2quaternion(initial_state.quat_body_eci);
quat_error =  conj(quat_cmd) * quat_body_eci;
if parts(quat_error) < 0
    quat_error = -quat_error;
end

[q1 q2 q3 q4] = utl_quaternion2array(quat_error);
actuators.wheel_torque = [q1; q2; q3]*const.Kp + initial_state.angular_rate_body*const.Kd;


end