function [state] = orb_apply_maneuver(state,self_maneuver_impulse_ecef,target_maneuver_impulse_ecef)
%ORB_APPLY_MANEUVER applies a suspected maneuver to update the estimate of the both orbits.
%   state(struct): 
%       state of the estimator described in orb_initialize_estimator
%   self_maneuver_impulse_ecef(matrix (3,1)): 
%       self maneuver impulse, should be zero, not NAN if no thrust (N*s)
%   target_maneuver_impulse_ecef(matrix (3,1)): 
%       target maneuver impulse, should be zero, not NAN if no thrust (N*s)
%

% Started by Nathan Zimmerberg on Feb 26, 2020
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Feb 26, 2020
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const
selfdv= self_maneuver_impulse_ecef/const.MASS;
targetdv= target_maneuver_impulse_ecef/const.MASS;
state.v_ecef= state.v_ecef+selfdv;
state.rel_target_v_ecef= state.rel_target_v_ecef-selfdv+targetdv;
self_v_covariance= norm(selfdv)*const.orb_self_thrust_noise_sdiv;
self_added_covariance= [zeros(3), zeros(3);
                        zeros(3), eye(3)*self_v_covariance^2];
state.self_covariance= state.self_covariance+self_added_covariance;
target_v_covariance= norm(targetdv)*const.orb_target_thrust_noise_sdiv;
target_added_covariance= [zeros(3), zeros(3);
                        zeros(3), eye(3)*target_v_covariance^2];
state.target_covariance= state.target_covariance+target_added_covariance;
end

