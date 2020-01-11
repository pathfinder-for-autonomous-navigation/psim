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
cycles_to_stable_mag_read= 5;
% how many cycles to a stable mag reading after a changed magrod command.
finite_diff_cycles_delay= 4;
% how long to wait to do finite diff on the mag readings.
switch state.counter
    case cycles_to_stable_mag_read
        %save magnetic field reading
        state.old_magnetic_field= magnetometer_body;
    case finite_diff_cycles_delay+cycles_to_stable_mag_read
        %calculate finite difference and bang bang controller
        deltaB= magnetometer_body-state.old_magnetic_field;
        state.magrod_moment_cmd= -sign(deltaB)*const.MAXMOMENT;
end
state.counter= mod(state.counter + 1,finite_diff_cycles_delay+cycles_to_stable_mag_read+1);
magrod_moment_cmd= state.magrod_moment_cmd;
end

