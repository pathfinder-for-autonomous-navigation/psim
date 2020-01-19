function state = adcs_mag_bias_est(state,SdotB_true,SdotB_measured,S)
%mag_bias_est runs the recursive least squares estimator
%   of the magentometer bias.
%   see https://en.wikipedia.org/wiki/Recursive_least_squares_filter#RLS_algorithm_summary
%   
%   state: state of the mag_bias_est contains,
%       p(matrix (3,3)): P matrix initialized to large value, see wiki article
%       b_bias_est(matrix (3,1)): 
%           estimated magnetometer bias (truth=measured-bias) in body frame (T)
%           initialized to zero.
%   SdotB_true(scalar): 
%       sun vector dot magnetic field vector from the models (T)
%   SdotB_measured(scalar): 
%       sun vector dot magnetic field vector from sensor measurements (T)
%   S(matrix (3,1)): sun vector in the body frame, unit vector.

% Started by Nathan Zimmerberg on Dec 31, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 5, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
P=state.p;
B_bias_est=state.b_bias_est;
SdotB_bias= SdotB_measured-SdotB_true;
g=P*S/(1+S'*P*S);
P= P-g*S'*P;
B_bias_est= B_bias_est + g*(SdotB_bias-dot(S,B_bias_est));
state.b_bias_est=B_bias_est;
state.p=P;
end

