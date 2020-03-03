function [sensors] = sensors_update(self_state,other_state)
%sensor_state_update updates sensor_states
%TODO update and define sensor biases
global const
self_dynamics= self_state.dynamics;
other_dynamics= other_state.dynamics;
sensors= self_state.sensors;
other_sensors= other_state.sensors;

pos_eci= get_truth('position eci',self_dynamics);
r_hat= pos_eci/norm(pos_eci);
gps_antenna= get_truth('antenna eci',self_dynamics);
if(dot(r_hat,gps_antenna)>=cos(const.gps_max_angle))
    sensors.gps_time_till_lock= sensors.gps_time_till_lock - double(const.dt)*1E-9;
else
    sensors.gps_time_till_lock= const.GPS_LOCK_TIME;
end

%check for cdgps lock
if (sensors.gps_time_till_lock<=0 && other_sensors.gps_time_till_lock<=0)
    relative_pos_eci=get_truth('position eci',other_dynamics)-pos_eci;
    relative_pos_eci_hat=relative_pos_eci/norm(relative_pos_eci);
    self_cdgps_antenna= get_truth('docking face eci',self_dynamics);
    other_cdgps_antenna= get_truth('docking face eci',other_dynamics);
    if (dot(self_cdgps_antenna,relative_pos_eci_hat)>=cos(const.cdgps_max_angle) ...
            && -dot(other_cdgps_antenna,relative_pos_eci_hat)>=cos(const.cdgps_max_angle)) ...
            && (const.cdgps_max_range>=norm(relative_pos_eci))
        sensors.cdgps_time_till_lock= sensors.cdgps_time_till_lock - double(const.dt)*1E-9;
    else
        sensors.cdgps_time_till_lock= const.CDGPS_LOCK_TIME;
    end
else
    sensors.cdgps_time_till_lock= const.CDGPS_LOCK_TIME;     
end

end

