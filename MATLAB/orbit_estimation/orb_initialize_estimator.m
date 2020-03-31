function [state] = orb_initialize_estimator()
%orb_initialize_estimator initializes the orbit estimator state.
%   The estimator is an EKF, inspired by the CanX-4/-5 algorithm.
%   https://tspace.library.utoronto.ca/bitstream/1807/25908/3/Roth_Niels_H_201011_MASc_thesis.pdf
% 
% state is a struct with the following elements:
%   target_stale(logical):
%       If true, the estimator will accept new ground readings.
%   time_of_last_cdgps(int64):
%       Time of last received cdgps (ns)
%       
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

