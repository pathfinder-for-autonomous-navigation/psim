function [state,magrod_moment_cmd,wheel_torque_cmd]=adcs_pointer(state,...
        angular_momentum_body, magnetic_field_body, ...
        primary_current_direction_body,...
        primary_desired_direction_body,...
        secondary_current_direction_body,...
        secondary_desired_direction_body,...
        rate_body)
% pointer PD controller for the wheels and magrods to point the primary and secondary
%   directions 
% This has a simple PD controller based on Justin's Simulink stuff.
% The magrods are controlled using a controller described in section 7.5.2 Momentum Dumping of 
%   Fundamentals of Spacecraft Attitude Determination and Control
% The difference, is that total momentum is used instead of just wheel
% momentum, and the gain is infinite.
%
%   if secondary pointing goal is NAN, the controller will just point the
%   primary.
%   for example:
%     primary_current_direction_body= estimator.sat2sun_body/norm(estimator.sat2sun_body);
%     primary_desired_direction_body= [1;0;0];
%     secondary_current_direction_body= NaN(3,1);
%     secondary_desired_direction_body= NaN(3,1);
%   will try and point the +X face at the sun.
%
%   
%   state(struct): state of the pointer contains, TBD
%   magrod_moment_cmd(matrix (3,1)): 
%       magrod moment cmd to output in body frame (A*m^2)
%   wheel_torque_cmd(matrix (3,1)): 
%       commanded wheel torque in body frame (N*m)
%   angular_momentum_body(matrix (3,1)): 
%       Total angular momentum of the satellite in the body frame(N*m*s)
%   magnetic_field_body(matrix (3,1)): (T)
%   primary_current_direction_body(matrix (3,1)):
%       these are the pointing commands, primary is optimized before secondary
%       everything must be a unit vector in the body frame, or NaN
%   primary_desired_direction_body(matrix (3,1)):
%       these are the pointing commands, primary is optimized before secondary
%       everything must be a unit vector in the body frame, or NaN
%   secondary_current_direction_body(matrix (3,1)):
%       these are the pointing commands, primary is optimized before secondary
%       everything must be a unit vector in the body frame, or NaN
%   secondary_desired_direction_body(matrix (3,1)):
%       these are the pointing commands, primary is optimized before secondary
%       everything must be a unit vector in the body frame, or NaN
%   rate_body(matrix (3,1)): angular rate of the body from eci in body (rad/s)


% Started by Nathan Zimmerberg on Dec 31, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 5, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const

%% Get Error %%
%get error, this is the first three terms of the error quaternion, if the
%forth component is positive.
error=zeros(3,1);
if all(isfinite([secondary_current_direction_body; secondary_desired_direction_body])) ...
        && abs(dot(secondary_current_direction_body,primary_current_direction_body)) < cos(10*pi/180) ...
        && abs(dot(secondary_desired_direction_body,primary_desired_direction_body)) < cos(10*pi/180)
    
    %in two pointing mode
    quat_desbody_body= utl_triad(primary_current_direction_body,secondary_current_direction_body, primary_desired_direction_body, secondary_desired_direction_body);
else
    %in one pointing mode
    quat_desbody_body=utl_vect_rot2quat(primary_desired_direction_body,primary_current_direction_body);
end
if quat_desbody_body(4)<0
    %make real component positive
    quat_desbody_body= -quat_desbody_body;
end
error= quat_desbody_body(1:3);

%% PD Controller %%
pterm= const.ATTITUDE_PD_KP*error;

% finite_diff=error-state.last_error;
% state.last_error=error;
% state.derivative_buffer(:,state.derivative_buffer_location+1)=finite_diff;
% state.derivative_buffer_location= mod(state.derivative_buffer_location+1,const.ATTITUDE_PD_derivative_buffer_size);
% if use_finite_diff
%     dterm= 2*const.ATTITUDE_PD_KD*median(state.derivative_buffer,2)/(double(const.dt)*1E-9);
% else
%     dterm= -const.ATTITUDE_PD_KD*rate_body;
% end
dterm= -const.ATTITUDE_PD_KD*rate_body;
wheel_torque_cmd= -(pterm+dterm);

%% Angular Momentum Control
if (norm(angular_momentum_body)>0.05*const.MAXWHEELRATE*const.JWHEEL)
    magrod_moment_cmd= sign(cross(angular_momentum_body,magnetic_field_body))*const.MAXMOMENT;
else
    magrod_moment_cmd= zeros(3,1);
end

end

