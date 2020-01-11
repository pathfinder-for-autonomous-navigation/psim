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
estimator.angular_rate_body= sensor_readings.gyro_body;
estimator.angular_momentum_body= const.JB*estimator.angular_rate_body+sensor_readings.wheel_momentum_body;
estimator.magnetometer_body= sensor_readings.magnetometer_body;
estimator.magnetic_field_body=estimator.magnetometer_body-state.mag_bias_est_state.b_bias_est;
estimator.quat_body_eci=utl_triad(estimator.sat2sun_eci,estimator.mag_eci,estimator.sat2sun_body,estimator.magnetic_field_body);
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
%everything must be a unit vector in the body frame, or NaN
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
        state.main_state='initialization hold';
    end 
end
if strcmp(state.main_state,'initialization hold')
    %move to next state if initialization time has passed
    if state.on_time>(30*60*1E9)
        state.main_state='detumble';
    end 
end
if strcmp(state.main_state,'detumble')
    %detumble
    if norm(estimator.angular_momentum_body)<const.MAXWHEELRATE*const.JWHEEL*const.detumble_safety_factor
        state.main_state='get gps';
        %on real sat send command to condition magnetometers here
    end 
end
return2detumble= estimator.tumbling || estimator.low_power;
if strcmp(state.main_state,'get gps')
    %check if gps is valid
    if return2detumble
        state.main_state='detumble';
    elseif estimator.valid_gps
        state.main_state="calibrate magnetometer 1";
    end
    %calculate pointing strategy
    state.primary_current_direction_body= estimator.sat2sun_body/norm(estimator.sat2sun_body);
    state.primary_desired_direction_body= [sqrt(2)/2;sqrt(2)/2;0];
    state.secondary_current_direction_body= NaN(3,1);
    state.secondary_desired_direction_body= NaN(3,1);
end
return2detumble= return2detumble || estimator.eclipse || ~estimator.valid_gps;
for i=1:3
    %These modes rotate the sat around to estimate the magnetometer bias. 
    %There are three modes, each one points a different axis at the sun, 
    %so the mag bias on that axis can be estimated accurately.
    if strcmp(state.main_state,"calibrate magnetometer "+i)
        axis= zeros(3,1);
        axis(i)= 1;
        if(estimator.sat2sun_body(i)>cos(20*pi/180))
            state.num_magnetometer_bias_readings(i)= state.num_magnetometer_bias_readings(i)+1;
        end
        if return2detumble
            state.main_state='detumble';
        elseif state.num_magnetometer_bias_readings(i)>=const.magnetometer_bias_readings_min
            if i<3
                state.main_state="calibrate magnetometer "+(i+1);
            else
                state.main_state= 'get target orbit';
            end
        end 
        %calculate pointing strategy
        state.primary_current_direction_body= estimator.sat2sun_body/norm(estimator.sat2sun_body);
        state.primary_desired_direction_body= axis; 
        state.secondary_current_direction_body= NaN(3,1);
        state.secondary_desired_direction_body= NaN(3,1);
    end   
end

if strcmp(state.main_state,'get target orbit')
    if return2detumble
        state.main_state='detumble';
    elseif estimator.valid_target_gps
        state.main_state='rendezvous';
    end 
    %calculate pointing strategy
    r_hat_eci= estimator.position_eci/norm(estimator.position_eci);
    r_hat_body= utl_rotateframe(estimator.quat_body_eci,r_hat_eci);
    r_hat_cross_sat2sun_body=cross(estimator.sat2sun_body,r_hat_eci);
    state.primary_current_direction_body= r_hat_body;
    state.primary_desired_direction_body= [1;0;0];
    if( norm(r_hat_cross_sat2sun_body) > sin(10*pi/180))
        state.secondary_current_direction_body= r_hat_cross_sat2sun_body/norm(r_hat_cross_sat2sun_body);
        state.secondary_desired_direction_body= [0;0;1];
    end
end
return2detumble= return2detumble || ~estimator.valid_target_gps;
if strcmp(state.main_state,'rendezvous')
    if return2detumble
        state.main_state='detumble';
    elseif norm(estimator.target_position_eci-estimator.position_eci)<0.1 && norm(estimator.target_velocity_eci-estimator.velocity_eci)<0.01
        state.main_state='docking';
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
if strcmp(state.main_state,'docking')
    state.main_state='docking';
end



%% Attitude Control %%
if strcmp(state.main_state,'detumble')
    if ~strcmp(previous_main_state,'detumble')
        %initialize detumbler
        initialize_detumble();
    end
    %detumbling
    [state.detumbler_state,actuators.magrod_moment]=adcs_detumbler(state.detumbler_state,sensor_readings.magnetometer_body);
elseif all(isfinite([state.primary_current_direction_body; state.primary_desired_direction_body]))
    %pointing mode
    if ~strcmp(previous_main_state,state.main_state)
        %initialize pointer if main_state changes
        state.pointer_state=adcs_initialize_pointer_state();
    end
    actuators.wheel_enable=true(3,1);
    [state.pointer_state,actuators.magrod_moment,actuators.wheel_torque]=adcs_pointer(state.pointer_state,...
        estimator.angular_momentum_body, estimator.magnetic_field_body, ...
        state.primary_current_direction_body,...
        state.primary_desired_direction_body,...
        state.secondary_current_direction_body,...
        state.secondary_desired_direction_body,...
        estimator.angular_rate_body);
else
    actuators.wheel_torque=zeros(3,1);%hold instead of stopping wheels 
end
if startsWith(state.main_state,"calibrate magnetometer ")
    if ~startsWith(previous_main_state,"calibrate magnetometer ")
        %initialize bias estimator
        state.mag_bias_est_state= adcs_initialize_mag_bias_est();
    end
    SdotB_true= dot(estimator.mag_eci, estimator.sat2sun_eci);
    SdotB_measured= dot(estimator.magnetometer_body, estimator.sat2sun_body);
    if all(isfinite([estimator.magnetometer_body,estimator.mag_eci,estimator.sat2sun_eci,estimator.sat2sun_body]))
        state.mag_bias_est_state=adcs_mag_bias_est(state.mag_bias_est_state,SdotB_true,SdotB_measured,estimator.sat2sun_body);
    end
end
state.on_time= state.on_time+ uint64(const.dt);

    function initialize_detumble()
        %initialize detumble helper
        state.detumbler_state=adcs_initialize_detumbler_state();
        state.num_magnetometer_bias_readings=zeros(3,1);
    end
end