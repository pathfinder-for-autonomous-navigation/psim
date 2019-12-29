function [state,magrod_moment_cmd,wheel_torque_cmd]=pointer(state,...
        angular_momentum_body, magnetic_field_body, ...
        primary_current_direction_body,...
        primary_desired_direction_body,...
        secondary_current_direction_body,...
        secondary_desired_direction_body,...
        rate_body,use_finite_diff)
%pointer PD controller for the wheels and magrods to point the primary and secondary
%directions
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

finite_diff=error-state.last_error;
state.last_error=error;
state.derivative_buffer(:,state.derivative_buffer_location+1)=finite_diff;
state.derivative_buffer_location= mod(state.derivative_buffer_location+1,const.ATTITUDE_PD_derivative_buffer_size);
if use_finite_diff
    dterm= 2*const.ATTITUDE_PD_KD*median(state.derivative_buffer,2)/(double(const.dt)*1E-9);
else
    dterm= -const.ATTITUDE_PD_KD*rate_body;
end
wheel_torque_cmd= -(pterm+dterm);

%% Angular Momentum Control
if (norm(angular_momentum_body)>0.05*const.MAXWHEELRATE*const.JWHEEL)
    magrod_moment_cmd= sign(cross(angular_momentum_body,magnetic_field_body))*const.MAXMOMENT;
else
    magrod_moment_cmd= zeros(3,1);
end

end

