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
state=initial_state;
actuators= struct();

%% Attitude determination

sat2sun_eci= env_sun_vector(sensor_readings.time);
mag_ecef= env_magnetic_field(sensor_readings.time,sensor_readings.position_ecef);
[quat_ecef_eci,~]=env_earth_attitude(sensor_readings.time);
quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
mag_eci =utl_rotateframe(quat_eci_ecef,mag_ecef);
quat_body_eci=utl_triad(sat2sun_eci,mag_eci,sensor_readings.sat2sun_body,sensor_readings.magnetometer_body);
angular_momentum_body= const.JB*sensor_readings.gyro_body+sensor_readings.wheel_momentum_body;


%% Contollers



switch state.adcs_state
    case 'init'
            actuators.magrod_moment= [0;0;0;];
            actuators.wheel_torque = [100;100;100;];
            actuators.wheel_enable=[0;0;0;];
            state.adcs_state='tumbling';
            state.detumbled_trigger_count=0;
            state.detumbler_state=0;
            state.magrod_moment_cmd= [0;0;0;];
    case 'tumbling'
        switch state.detumbler_state
            case 5
                %save magnetic field reading
                state.old_magnetic_field= sensor_readings.magnetometer_body;
            case 9
                %calculate finite difference and bang bang controller
                deltaB= sensor_readings.magnetometer_body-state.old_magnetic_field;
                state.magrod_moment_cmd= -sign(deltaB)*const.MAXMOMENT;
        end
        state.detumbler_state= mod(state.detumbler_state + 1,10);
        actuators.magrod_moment= state.magrod_moment_cmd;
        actuators.wheel_torque = [100;100;100;];
        actuators.wheel_enable=[0;0;0;];
        if (norm(angular_momentum_body)<0.2*const.MAXWHEELRATE*const.JWHEEL)
            %detumble done detected, increase detumbled trigger count
            state.detumbled_trigger_count = state.detumbled_trigger_count+1;
        else
            state.detumbled_trigger_count = 0;
        end
        if (state.detumbled_trigger_count>=2)
            %detumbled triggered twice in a row
            state.adcs_state='pointing';
            state.tumbling_trigger_count=0;
        end
    case 'pointing'
        %TODO calculate where to point
        quat_cmd = [0;0;1;0;];
        quat_error =  utl_quat_cross_mult(quat_body_eci,utl_quat_conj(quat_cmd));
        if quat_error(4) < 0 %make sure the forth componet is positive
            quat_error = -quat_error;
        end
        q = quat_error;
        actuators.wheel_torque = q(1:3)*const.ATTITUDE_PD_KP + sensor_readings.gyro_body*const.ATTITUDE_PD_KD;
        actuators.wheel_enable=[1;1;1;];
        if (norm(angular_momentum_body)>0.1*const.MAXWHEELRATE*const.JWHEEL)
            actuators.magrod_moment= sign(cross(angular_momentum_body,sensor_readings.magnetometer_body))*const.MAXMOMENT;
        else
            actuators.magrod_moment= [0;0;0;];
        end
        if (norm(angular_momentum_body)>0.7*const.MAXWHEELRATE*const.JWHEEL)
            %tumbling detected, increase tumbling trigger count
            disp('tumbling detected')
            state.tumbling_trigger_count = state.tumbling_trigger_count+1;
        else
            state.tumbling_trigger_count = 0;
        end
        if (state.tumbling_trigger_count>=2)
            %tumbling triggered twice in a row
            state.adcs_state='tumbling';
            state.detumbled_trigger_count = 0;
            state.detumbler_state=0;
            state.magrod_moment_cmd= [0;0;0;];
        end
    otherwise
        actuators.magrod_moment= [0;0;0;];
        actuators.wheel_torque = [100;100;100;];
        actuators.wheel_enable=[0;0;0;];
end
final_state=state;
end