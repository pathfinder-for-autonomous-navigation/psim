function [adcs_state,magrod_moment,wheel_torque,wheel_enable]=adcs_update(adcs_state, ...
    angular_momentum_body,...
    angular_rate_body, ...
    orbit, ...
    target_orbit, ...
    quat_body_eci, ...
    magnetometer_body, ...
    magnetic_field_body, ...
    sat2sun_body, ...
    sat2sun_eci, ...
    next_main_state)
% ADCS_UPDATE
%   updates the attitude control state, and calculates actuators based on
%   mode
% Contollers
global const


if (~strcmp(adcs_state.main_state,next_main_state))
    switch next_main_state
        case 'Off'
            initOff();
        case 'Detumble'
            initDetumble();
        case 'Standby'
            initStandby();
    end
end

switch next_main_state
    case 'Off'
        doOff();
    case 'Detumble'
        doDetumble();
    case 'Standby'
        doStandby();
end
    
    function initOff()
    end
    
    function doOff()
        magrod_moment= [0;0;0;];
        wheel_torque = [100;100;100;];
        wheel_enable=[0;0;0;];
    end


    function initDetumble()
        adcs_state.detumbler_state=0;
        adcs_state.magrod_moment_cmd= [0;0;0;];
    end

    function doDetumble()
        switch adcs_state.detumbler_state
            case 5
                %save magnetic field reading
                adcs_state.old_magnetic_field= magnetometer_body;
            case 9
                %calculate finite difference and bang bang controller
                deltaB= magnetometer_body-adcs_state.old_magnetic_field;
                adcs_state.magrod_moment_cmd= -sign(deltaB)*const.MAXMOMENT;
        end
        adcs_state.detumbler_state= mod(adcs_state.detumbler_state + 1,10);
        magrod_moment= adcs_state.magrod_moment_cmd;
        wheel_torque = [100;100;100;];
        wheel_enable=[0;0;0;];
    end

    function initStandby()
    end

    function doStandby()
        %check if things are valid, the estimator is in charge of setting
        %things to NaN or +-inf if they aren't valid.
        r_eci= orbit(1:3);
        r_eci_hat= r_eci/norm(r_eci); %+x face
        r_cross_s = cross(r_eci_hat,sat2sun_eci);
        r_cross_s_hat= r_cross_s/norm(r_cross_s); %+z face
        y_bodydesired_eci = cross(r_cross_s_hat,r_eci_hat); %+y face
        dcm_eci_bodydesired= [r_eci_hat y_bodydesired_eci r_cross_s_hat];
        quat_eci_bodydesired= utl_dcm2quat(dcm_eci_bodydesired);
        quat_body_bodydesired= utl_quat_cross_mult(quat_body_eci,quat_eci_bodydesired);
        if (~ all(isfinite(quat_body_bodydesired)))
            %something got messed up, 
            %try and point antennas at sun sensor reading.
            quat_body_bodydesired= utl_vect_rot2quat(sat2sun_body,[1;0;0;]);
        end
        quat_error=quat_body_bodydesired;
        if quat_error(4) < 0 %make sure the forth componet is positive
             quat_error = -quat_error;
        end
        q = quat_error;
        wheel_torque = q(1:3)*const.ATTITUDE_PD_KP + angular_rate_body*const.ATTITUDE_PD_KD;
        wheel_enable=[1;1;1;];
        if (norm(angular_momentum_body)>0.1*const.MAXWHEELRATE*const.JWHEEL)
            magrod_moment= sign(cross(angular_momentum_body,magnetic_field_body))*const.MAXMOMENT;
        else
            magrod_moment= [0;0;0;];
        end
    end

end
        
        
