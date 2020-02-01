function [state,self2target_r_ecef,self2target_v_ecef,r_ecef,v_ecef] ...
    = orb_run_estimator(...
    state,maneuver,fixedrtk,reset_all,reset_target,gps_r_ecef,gps_v_ecef,gps_self2target_r_ecef,time_ns,ground_target_r_ecef,ground_target_v_ecef,ground_time_ns)
%ORB_RUN_ESTIMATOR updates the estimate of the both orbits and outputs the needed ADCS values.
global const
%handle resets
if (reset_all)
    state = orb_initialize_estimator();
end
if (reset_target)
    state.target_stale= true;
    state.rel_target_r_ecef= nan(3,1);
    state.rel_target_v_ecef= nan(3,1);
    state.target_covariance= eye(6)*1E16;
    state.self_target_cross_covariance= zeros(6,6);
    state.time_of_last_cdgps= -int64(1E18);
    state.saved_ground_target_r_ecef= nan(3,1);
    state.saved_ground_target_v_ecef= nan(3,1);
    state.saved_ground_time_ns= int64(0);
end
%determine gps validity 
%TODO add better sanity checks
gpsvalid= all(isfinite([gps_r_ecef;gps_v_ecef;]));
groundvalid= all(isfinite([ground_target_r_ecef;ground_target_v_ecef;]));
saved_groundvalid= all(isfinite([state.saved_ground_target_r_ecef;state.saved_ground_target_v_ecef;]));
cdgpsvalid= gpsvalid && all(isfinite(gps_self2target_r_ecef));
selfvalid= all(isfinite([state.r_ecef;state.v_ecef;])) ...
    && all(isfinite(state.self_covariance),'all');
targetvalid= selfvalid && all(isfinite([state.rel_target_r_ecef;state.rel_target_v_ecef])) ...
    && all(isfinite(state.target_covariance),'all') ...
    && all(isfinite(state.self_target_cross_covariance),'all');
%handle initializations
if (~selfvalid && gpsvalid)
    "self init"
    state= init_self_orbit_gps(state,gps_r_ecef,gps_v_ecef,time_ns);
    selfvalid= true;
end

if ((~targetvalid || state.target_stale) && selfvalid && groundvalid)
    "target init"
    state= init_target_orbit_ground(state,ground_target_r_ecef,ground_target_v_ecef,ground_time_ns);
    targetvalid= true;
    state.target_stale= true;
end

if (~targetvalid && selfvalid && saved_groundvalid)
    "target init from saved"
    state= init_target_orbit_ground(state,ground_target_r_ecef,ground_target_v_ecef,ground_time_ns);
    targetvalid= true;
    state.target_stale= true;
    state.saved_ground_target_r_ecef= nan(3,1);
    state.saved_ground_target_v_ecef= nan(3,1);
    state.saved_ground_time_ns= int64(0);
end

if (groundvalid && ~selfvalid)
    "save ground"
    state.saved_ground_target_r_ecef= ground_target_r_ecef;
    state.saved_ground_target_v_ecef= ground_target_v_ecef;
    state.saved_ground_time_ns= ground_time_ns;
end

%propagate state and correct with measurements
if (cdgpsvalid && targetvalid)
    state.target_stale= false;
    state.time_of_last_cdgps= time_ns;
    state = update_with_cdgps(state,fixedrtk,gps_r_ecef,gps_v_ecef,gps_self2target_r_ecef,time_ns);
elseif (gpsvalid && targetvalid)
    state = update_with_noncdgps(state,gps_r_ecef,gps_v_ecef,time_ns);
elseif (gpsvalid && selfvalid)
    state = update_with_noncdgps_invalid_target(state,gps_r_ecef,gps_v_ecef,time_ns);
elseif (selfvalid)
    %move state to time_ns
    while(time_ns~=state.time_ns)
        dt= min(max((time_ns-state.time_ns),int64(-2E8)),int64(2E8));
        state = propagate_state(state,dt);
    end
end

if ((time_ns-state.time_of_last_cdgps) > const.time_for_stale_cdgps)
    state.target_stale= true;
end

%output
self2target_r_ecef= state.rel_target_r_ecef;
self2target_v_ecef= state.rel_target_v_ecef;
r_ecef= state.r_ecef;
v_ecef= state.v_ecef;
end

function [state] = propagate_state(state,dt)
    %propagate_state, update state
    %   dt(int64 -0.2E9 to 0.2E9):
    %       Time step, how much time to update the state (ns)
    global const
    bad_force_related_var= const.orb_process_noise_var;%this relates to added variance for bad force models
    state.self_covariance= state.self_covariance + 0.5*double(dt)*1E-9*bad_force_related_var;
    state.target_covariance= state.target_covariance + 0.5*double(dt)*1E-9*bad_force_related_var;
    [state.r_ecef,state.v_ecef,jacobian,state.rel_target_r_ecef,state.rel_target_v_ecef,target_jacobian] = ...
        orb_short_orbit_prop(...
        state.r_ecef,state.v_ecef,state.rel_target_r_ecef,state.rel_target_v_ecef,double(dt)*1E-9,double(state.time_ns)*1E-9);
    state.self_covariance= jacobian*state.self_covariance*jacobian';
    state.target_covariance= target_jacobian*state.self_covariance*target_jacobian';
    state.self_target_cross_covariance= jacobian*state.self_target_cross_covariance*target_jacobian';
    state.time_ns= state.time_ns + dt;
    state.target_covariance= state.target_covariance + 0.5*double(dt)*1E-9*bad_force_related_var;
    state.self_covariance= state.self_covariance + 0.5*double(dt)*1E-9*bad_force_related_var;
end

function [state] = update_with_noncdgps(state,gps_r_ecef,gps_v_ecef,time_ns)
    %move state to time_ns
    global const
    while(time_ns~=state.time_ns)
        dt= min(max((time_ns-state.time_ns),int64(-2E8)),int64(2E8));
        state = propagate_state(state,dt);
    end
    %measurement correction.
    y= [gps_r_ecef;gps_v_ecef;];
    H= [eye(6), zeros(6)];
    x= [state.r_ecef;state.v_ecef;state.rel_target_r_ecef+state.r_ecef;state.rel_target_v_ecef+state.v_ecef;];
    P= [state.self_covariance, state.self_target_cross_covariance;
        state.self_target_cross_covariance', state.target_covariance;];
    R= const.single_gps_noise_covariance;% noise covariance
    [x,P] = kalman_correction_step(x,y,H,P,R);
    state.r_ecef= x(1:3);
    state.v_ecef= x(4:6);
    state.rel_target_r_ecef= x(7:9)-x(1:3);
    state.rel_target_v_ecef= x(10:12)-x(4:6);
    state.self_covariance= P(1:6,1:6);
    state.self_target_cross_covariance= P(1:6,7:12);
    state.target_covariance= P(7:12,7:12);
end

function [state] = update_with_noncdgps_invalid_target(state,gps_r_ecef,gps_v_ecef,time_ns)
    %move state to time_ns
    global const
    while(time_ns~=state.time_ns)
        dt= min(max((time_ns-state.time_ns),int64(-2E8)),int64(2E8));
        state = propagate_state(state,dt);
    end
    %measurement correction.
    y= [gps_r_ecef;gps_v_ecef;];
    H= eye(6);
    x= [state.r_ecef;state.v_ecef;];
    P= state.self_covariance;
    R= const.single_gps_noise_covariance;% noise covariance
    [x,P] = kalman_correction_step(x,y,H,P,R);
    state.r_ecef= x(1:3);
    state.v_ecef= x(4:6);
    state.self_covariance= P;
end

function [state] = update_with_cdgps(state,fixedrtk,gps_r_ecef,gps_v_ecef,gps_self2target_r_ecef,time_ns)
    %move state to time_ns and correct state
    global const
    while(time_ns~=state.time_ns)
        dt= min(max((time_ns-state.time_ns),int64(-2E8)),int64(2E8));
        state = propagate_state(state,dt);
    end
    %measurement correction.
    y= [gps_r_ecef;gps_v_ecef;gps_self2target_r_ecef];
    H= [eye(6), zeros(6);
        -eye(3), zeros(3), eye(3), zeros(3)];
    x= [state.r_ecef;state.v_ecef;state.rel_target_r_ecef+state.r_ecef;state.rel_target_v_ecef+state.v_ecef;];
    P= [state.self_covariance, state.self_target_cross_covariance;
        state.self_target_cross_covariance', state.target_covariance;];
    if fixedrtk
        R= const.fixed_cdgps_noise_covariance;% noise covariance
    else
        R= const.float_cdgps_noise_covariance;% noise covariance
    end
    [x,P] = kalman_correction_step(x,y,H,P,R);
    state.r_ecef= x(1:3);
    state.v_ecef= x(4:6);
    state.rel_target_r_ecef= x(7:9)-x(1:3);
    state.rel_target_v_ecef= x(10:12)-x(4:6);
    state.self_covariance= P(1:6,1:6);
    state.self_target_cross_covariance= P(1:6,7:12);
    state.target_covariance= P(7:12,7:12);
end

function [state] = init_target_orbit_ground(state,target_r_ecef,target_v_ecef,time_ns)
    global const
    state.self_target_cross_covariance= zeros(6,6);
    state.target_covariance= const.single_gps_noise_covariance; %initialize covariance
    %propagate target forward
    delta_t_ns= state.time_ns-time_ns;
    [target_r_ecef,target_v_ecef] = orb_long_orbit_prop(target_r_ecef, target_v_ecef, double(delta_t_ns)*1E-9, double(time_ns)*1E-9);
    state.rel_target_r_ecef= target_r_ecef-state.r_ecef;
    state.rel_target_v_ecef= target_v_ecef-state.v_ecef;
end

function [state] = init_self_orbit_gps(state,gps_r_ecef,gps_v_ecef,time_ns)
    global const
    state.self_target_cross_covariance= zeros(6,6);
    state.self_covariance= const.single_gps_noise_covariance; %initialize covariance
    state.time_ns=time_ns;
    state.r_ecef= gps_r_ecef;
    state.v_ecef= gps_v_ecef;
end

function [x,P] = kalman_correction_step(x,y,H,P,R)
    % correction step of kalman filter, see canx paper eq 4.4 - 4.7
    %get kalman gain K
    K= P*H'*inv(H*P*H'+ R);
    P= (eye(length(x))-K*H)*P;
    x= x + K*(y-H*x);
end