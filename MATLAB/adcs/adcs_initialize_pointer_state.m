function [state] = adcs_initialize_pointer_state()
%initialize_pointer_state Initializes the pointer state
%   
%   state: state of the pointer contains, TBD

% Started by Nathan Zimmerberg on Dec 31, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 5, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const
state=struct();
%Initialize moving median filter
%state.derivative_buffer=zeros(3,const.ATTITUDE_PD_derivative_buffer_size);
%state.derivative_buffer_location=0;
%state.last_error= zeros(3,1);
end

