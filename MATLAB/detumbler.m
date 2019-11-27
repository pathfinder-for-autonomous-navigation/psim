function [state,magrod_moment_cmd] = detumbler(state,magnetometer_body)
%detumbler Uses the Bdot control law to detumble the satellite
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

