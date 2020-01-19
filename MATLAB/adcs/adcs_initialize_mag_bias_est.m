function state = adcs_initialize_mag_bias_est()
%initialize_mag_bias_est initializes the recursive least squares estimator
%   of the magentometer bias.
%   see https://en.wikipedia.org/wiki/Recursive_least_squares_filter#RLS_algorithm_summary
%   
%   state: state of the mag_bias_est contains,
%       p(matrix (3,3)): P matrix initialized to large value, see wiki article
%       b_bias_est(matrix (3,1)): 
%           estimated magnetometer bias (truth=measured-bias) in body frame (T)
%           initialized to zero.

% Started by Nathan Zimmerberg on Dec 31, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 5, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
state.p= eye(3)*1E4;
state.b_bias_est= [0;0;0;];
end

