function [state,self2target_pos_eci,earth2self_pos_eci,self_orbit,target_orbit] = orb_run_estimator(state,maneuver_happened,gpsmode,self_gpsdata,target_orbit_ground_est)
%ORB_RUN_ESTIMATOR updates the estimate of the both orbits and outputs the needed ADCS values.

end

function [state] = propagate_state(state,dt)
%propagate_state, update state
%   dt(int64 -0.2E9 to 0.2E9):
%       Time step, how much time to update the state (ns)
bad_force_related_var= eye(6)*0;%this relates to added variance for bad force models
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

function [state] = update_with_cdgps(state,)
end

function [state] = update_with_noncdgps(state,self_gpsdata)
end

function [state] = init_target_orbit_ground(state,)
end

function [state] = init_self_orbit_gps(state,)
end