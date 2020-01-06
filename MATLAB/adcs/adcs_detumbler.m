function [state,magrod_moment_cmd] = adcs_detumbler(state,magnetometer_body)
%adcs_detumbler Uses the Bdot control law to detumble the satellite.
%   This is described in section 7.5.1 Detumbling of 
%   Fundamentals of Spacecraft Attitude Determination and Control
%
%   state(struct): state of the adcs_detumbler contains,
%       counter(int): a repeating counter that ensure proper timing.
%       old_magnetic_field(matrix (3,1)): 
%           a saved magnetometer reading in body frame (T)
%       magrod_moment_cmd(matrix (3,1)):
%           the saved magrod moment cmd to output in body frame (A*m^2)
%   magrod_moment_cmd(matrix (3,1)): 
%       magrod moment cmd to output in body frame (A*m^2)
%   magnetometer_body(matrix (3,1)): 
%       a magnetometer reading in body frame (T)

% Started by Nathan Zimmerberg on Nov 27, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Dec 31, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const
switch state.counter
    case 5
        %save magnetic field reading
        state.old_magnetic_field= magnetometer_body;
    case 9
        %calculate finite difference and bang bang controller
        deltaB= magnetometer_body-state.old_magnetic_field;
        state.magrod_moment_cmd= -sign(deltaB)*const.MAXMOMENT;
end
state.counter= mod(state.counter + 1,10);
magrod_moment_cmd= state.magrod_moment_cmd;
end

