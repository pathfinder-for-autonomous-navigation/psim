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
actuators= struct();

% Attitude Determination %

sat2sun_eci= env_sun_vector(sensor_readings.time);
mag_ecef= env_magnetic_field(sensor_readings.time,sensor_readings.position_ecef);
[quat_ecef_eci,~]=env_earth_attitude(sensor_readings.time);
quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
mag_eci =utl_rotateframe(quat_eci_ecef,mag_ecef);
quat_body_eci=utl_triad(sat2sun_eci,mag_eci,sensor_readings.sat2sun_body,sensor_readings.magnetometer_body);
angular_rate_body= sensor_readings.gyro_body;
angular_momentum_body= const.JB*angular_rate_body+sensor_readings.wheel_momentum_body;
sat2sun_body= sensor_readings.sat2sun_body;
magnetometer_body= sensor_readings.magnetometer_body;
magnetic_field_body=magnetometer_body;
orbit=NaN(6,1);
target_orbit=NaN(6,1);



% Attitude Main State Determination %
switch state.adcs_state.main_state
    case 'Off'
        next_adcs_main_state= 'Detumble';
        state.detumbled_trigger_count = 0;
    case 'Detumble'
        if (norm(angular_momentum_body)<0.2*const.MAXWHEELRATE*const.JWHEEL)
            %detumble done detected, increase detumbled trigger count
            state.detumbled_trigger_count = state.detumbled_trigger_count+1;
        else
            state.detumbled_trigger_count = 0;
        end
        if (state.detumbled_trigger_count>=2)
            %detumbled triggered twice in a row
            next_adcs_main_state='Standby';
            state.tumbling_trigger_count=0;
        else
            next_adcs_main_state='Detumble';
        end
    case 'Standby'
        if (norm(angular_momentum_body)>0.7*const.MAXWHEELRATE*const.JWHEEL)
            %tumbling detected, increase tumbling trigger count
            state.tumbling_trigger_count = state.tumbling_trigger_count+1;
        else
            state.tumbling_trigger_count = 0;
        end
        if (state.tumbling_trigger_count>=2)
            %tumbling triggered twice in a row
            next_adcs_main_state='Detumble';
            state.detumbled_trigger_count = 0;
        else 
            next_adcs_main_state='Standby';
        end
        
end


% Attitude Control %
[state.adcs_state,actuators.magrod_moment,actuators.wheel_torque,actuators.wheel_enable]=adcs_update(state.adcs_state, ...
    angular_momentum_body,...
    angular_rate_body, ...
    orbit, ...
    target_orbit, ...
    quat_body_eci, ...
    magnetometer_body, ...
    magnetic_field_body, ...
    sat2sun_body, ...
    sat2sun_eci, ...
    next_adcs_main_state);
end