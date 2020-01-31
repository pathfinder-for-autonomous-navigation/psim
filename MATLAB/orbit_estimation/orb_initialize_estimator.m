function [state] = orb_initialize_estimator()
%orb_initialize_estimator initializes the orbit estimator state.
% 
% state is a struct with the following elements:
%   mode(string either "propagate" or "cd gps"):
%       The mode of the estimator for the target orbit. 
state=struct();
state.target_stale= true;
state.time_of_last_cdgps= -int64(1E18);
state.saved_ground_target_r_ecef= nan(3,1);
state.saved_ground_target_v_ecef= nan(3,1);
state.saved_ground_time_ns= int64(0);
state.r_ecef= nan(3,1);
state.v_ecef= nan(3,1);
state.rel_target_r_ecef= nan(3,1);
state.rel_target_v_ecef= nan(3,1);
state.time_ns= int64(0);
state.self_covariance= eye(6)*1E16;
state.target_covariance= eye(6)*1E16;
state.self_target_cross_covariance= zeros(6,6);
end

