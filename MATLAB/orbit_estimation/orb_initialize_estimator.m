function [state] = orb_initialize_estimator()
%orb_initialize_estimator initializes the orbit estimator state.
% 
% state is a struct with the following elements:
%   mode(string either "propagate" or "cd gps"):
%       The mode of the estimator for the target orbit. 
state=struct();
state.mode= "propagate";
end

