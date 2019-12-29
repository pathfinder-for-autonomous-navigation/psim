function [state,actuators] = update_FC_state(state,sensor_readings)
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

%% Attitude Determination %%

estimator.sat2sun_eci= env_sun_vector(sensor_readings.time);
estimator.mag_ecef= env_magnetic_field(sensor_readings.time,sensor_readings.position_ecef);
[estimator.quat_ecef_eci,~]=env_earth_attitude(sensor_readings.time);
estimator.quat_eci_ecef= utl_quat_conj(estimator.quat_ecef_eci);
estimator.mag_eci =utl_rotateframe(estimator.quat_eci_ecef,estimator.mag_ecef);
estimator.sat2sun_body= sensor_readings.sat2sun_body;
estimator.quat_body_eci=utl_triad(estimator.sat2sun_eci,estimator.mag_eci,sensor_readings.sat2sun_body,sensor_readings.magnetometer_body);
estimator.angular_rate_body= sensor_readings.gyro_body;
estimator.angular_momentum_body= const.JB*estimator.angular_rate_body+sensor_readings.wheel_momentum_body;
estimator.magnetometer_body= sensor_readings.magnetometer_body;
estimator.magnetic_field_body=estimator.magnetometer_body;
estimator.time= sensor_readings.time;
estimator.position_eci= utl_rotateframe(estimator.quat_eci_ecef,sensor_readings.position_ecef);
estimator.velocity_eci= utl_rotateframe(estimator.quat_eci_ecef,sensor_readings.velocity_ecef+cross(const.earth_rate_ecef,sensor_readings.position_ecef));
estimator.target_position_eci=utl_rotateframe(estimator.quat_eci_ecef,sensor_readings.target_position_ecef);
estimator.target_velocity_eci=utl_rotateframe(estimator.quat_eci_ecef,sensor_readings.target_velocity_ecef+cross(const.earth_rate_ecef,sensor_readings.target_position_ecef));
estimator.tumbling= ~(norm(estimator.angular_momentum_body)<const.MAXWHEELRATE*const.JWHEEL*0.7);
estimator.low_power= false;
estimator.eclipse= env_eclipse(estimator.position_eci,estimator.sat2sun_eci);
estimator.valid_gps=all(isfinite([estimator.position_eci; estimator.velocity_eci; estimator.time]));
estimator.valid_target_gps=estimator.valid_gps && all(isfinite([estimator.target_position_eci; estimator.target_velocity_eci; estimator.time]));

if estimator.valid_gps && ~estimator.eclipse && ~all(isfinite(estimator.sat2sun_body))
    estimator.sat2sun_body= [0;0;-1]; % assume the sun is coming from the docking face
    estimator.quat_body_eci=utl_triad(estimator.sat2sun_eci,estimator.mag_eci,estimator.sat2sun_body,estimator.magnetic_field_body);
end


%% Get main_state %%
% by default turn off actuators and adcs control
actuators= actuators_off_command();

%these are the pointing commands, primary is optimized before secondary
%everythin must be a unit vector in the body frame, or NaN
state.primary_current_direction_body=NaN(3,1);
state.primary_desired_direction_body=NaN(3,1);
state.secondary_current_direction_body=NaN(3,1);
state.secondary_desired_direction_body=NaN(3,1);


previous_main_state=state.main_state;
if strcmp(state.main_state,'off')
    state.main_state='hardware setup';
end
if strcmp(state.main_state,'hardware setup')
    %move to next state if all sub systems are setup
    if true
        state.on_time=uint64(0);
        state.main_state='initialization hold';
    end 
end
if strcmp(state.main_state,'initialization hold')
    %move to next state if initialization time has passed
    if state.initialization_hold_done || state.on_time>(30*60*1E9)
        state.initialization_hold_done=true;%save this in eprom
        state.main_state='detumble';
    end 
end
if strcmp(state.main_state,'detumble')
    %detumble
    if norm(estimator.angular_momentum_body)<const.MAXWHEELRATE*const.JWHEEL*0.2
        state.main_state='get gps';
    end 
end
if strcmp(state.main_state,'get gps')
    %check if gps is valid
    if estimator.tumbling==1 || estimator.low_power==1
        state.main_state='detumble';
    elseif estimator.valid_gps
        state.main_state='get target orbit';
    end
    %calculate pointing strategy
    state.primary_current_direction_body= estimator.sat2sun_body/norm(estimator.sat2sun_body);
    state.primary_desired_direction_body= [1;0;0];
    state.secondary_current_direction_body=NaN(3,1);
    state.secondary_desired_direction_body=NaN(3,1);
end
if strcmp(state.main_state,'get target orbit')
    if estimator.tumbling || estimator.low_power || estimator.eclipse || ~estimator.valid_gps
        state.main_state='detumble';
    elseif estimator.valid_target_gps
        state.main_state='rendezvous';
    end 
    %calculate pointing strategy
    r_hat_eci= estimator.position_eci/norm(estimator.position_eci);
    v_hat_eci= estimator.velocity_eci/norm(estimator.velocity_eci);
    r_hat_body= utl_rotateframe(estimator.quat_body_eci,r_hat_eci);
    v_hat_body= utl_rotateframe(estimator.quat_body_eci,v_hat_eci);
    r_hat_cross_sat2sun_body=cross(estimator.sat2sun_body,r_hat_eci);
    state.primary_current_direction_body= r_hat_body;
    state.primary_desired_direction_body= [1;0;0];
    state.secondary_current_direction_body= r_hat_cross_sat2sun_body/norm(r_hat_cross_sat2sun_body);
    state.secondary_desired_direction_body= [0;0;1];
end
if strcmp(state.main_state,'rendezvous')
    if estimator.tumbling || estimator.low_power || estimator.eclipse || ~estimator.valid_gps || ~estimator.valid_target_gps
        state.main_state='detumble';
    elseif norm(estimator.target_position_eci-estimator.position_eci)<0.1 && norm(estimator.target_velocity_eci-estimator.velocity_eci)<0.01
        state.main_state='????';
    end 
    %calculate pointing strategy
    r_hat_eci= estimator.position_eci/norm(estimator.position_eci);
    sat2target_r_eci= -estimator.position_eci+estimator.target_position_eci;
    sat2target_r_eci_hat= sat2target_r_eci/norm(sat2target_r_eci);
    sat2target_r_body_hat= utl_rotateframe(estimator.quat_body_eci,sat2target_r_eci_hat);
    r_hat_body= utl_rotateframe(estimator.quat_body_eci,r_hat_eci);
    r_hat_cross_sat2sun_body=cross(estimator.sat2sun_body,r_hat_eci);
    state.primary_current_direction_body= r_hat_body;
    state.primary_desired_direction_body= [1;0;0];
    state.secondary_current_direction_body= sat2target_r_body_hat;
    state.secondary_desired_direction_body= [0;0;-1];
end
if strcmp(state.main_state,'????')
    state.main_state='PROFIT';
end

%% Attitude Control %%
if strcmp(state.main_state,'detumble')
    if ~strcmp(previous_main_state,'detumble')
        %initialize detumbler
        state.detumbler_state=initialize_detumbler_state();
    end
    %detumbling
    [state.detumbler_state,actuators.magrod_moment]=detumbler(state.detumbler_state,sensor_readings.magnetometer_body);
elseif all(isfinite([state.primary_current_direction_body; state.primary_desired_direction_body]))
    %pointing mode
    if ~strcmp(previous_main_state,state.main_state)
        %initialize pointer if main_state changes
        state.pointer_state=initialize_pointer_state();
    end
    actuators.wheel_enable=true(3,1);
    [state.pointer_state,actuators.magrod_moment,actuators.wheel_torque]=pointer(state.pointer_state,...
        estimator.angular_momentum_body, estimator.magnetic_field_body, ...
        state.primary_current_direction_body,...
        state.primary_desired_direction_body,...
        state.secondary_current_direction_body,...
        state.secondary_desired_direction_body,...
        estimator.angular_rate_body,true);
else
    actuators.wheel_torque=zeros(3,1);%hold instead of stopping wheels 
end
state.on_time= state.on_time+ uint64(const.dt);

end