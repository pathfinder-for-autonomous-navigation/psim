function [state] = adcs_initialize_detumbler_state()
%adcs_initialize_detumbler_state Initializes the detumbler state
%   This is described in section 7.5.1 Detumbling of 
%   Fundamentals of Spacecraft Attitude Determination and Control
%   
%   state: state of the adcs_detumbler contains,
%       counter(int): a repeating counter that ensure proper timing.
%       old_magnetic_field(matrix (3,1)): 
%           a saved magnetometer reading in body frame (T)
%       magrod_moment_cmd(matrix (3,1)):
%           the saved magrod moment cmd to output in body frame (A*m^2)

% Started by Nathan Zimmerberg on Nov 27, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Dec 31, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
state.counter=0;
state.old_magnetic_field=zeros(3,1);
state.magrod_moment_cmd= zeros(3,1);
end

