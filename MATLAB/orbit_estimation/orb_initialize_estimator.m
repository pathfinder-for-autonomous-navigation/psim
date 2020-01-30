function [state] = orb_initialize_estimator()
%orb_initialize_estimator initializes the orbit estimator state.
% 
% state is a struct with the following elements:
%   mode(string either "propagate" or "cd gps"):
%       The mode of the estimator for the target orbit. 
state=struct();
state.mode= "propagate";
state.r_ecef= nan(3,1);
state.v_ecef= nan(3,1);
state.rel_target_r_ecef= nan(3,1);
state.rel_target_v_ecef= nan(3,1);
state.time_ns= int64(0);
state.self_covariance= eye(6)*inf;
state.target_covariance= eye(6)*inf;
state.self_target_cross_covariance= zeros(6,6);
end

