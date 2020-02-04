% Test script to check the orbit propegator's ode solver RHS function (in this
% script that's `j4`, works correctly)
%
% This propegator works stricly in a j4 model and therefore doesn't need to
% worry about the conversion from ECI to ECEF. If you have questions about what
% the `j4` function is actually doing, talk to Kyle - yes, I am typing in the
% third person ;)
%
% When running it, you should be able to see the precission about the z-axis
% happening.
%

a= 6860636.6;       % semimajor axis
e= 0.001;           % eccentricity
p= a*(1-e);         % semilatus rectum
i_LDR= 45*pi/180;   % inclination angle
i_FWR= 45*pi/180;   % inclination angle
O=0;                % right ascension of the ascending node (longitude)
o=0;                % Argument of Perigee
nu_LDR=0*pi/180;    % True anamoly
nu_FWR=0*pi/180;    % True anamoly
[r, v] = utl_orb2rv(p, e, i_LDR, O, o, nu_LDR, 3.986e14);

opt = odeset('RelTol', 1e-10, 'AbsTol', 1e-2, 'InitialStep', 0.1);
sol = ode45(@j4, [0.0, 50.0 * 3600.0], [r; v], opt);
plot(sol.y(1, :), sol.y(2, :))

% This function will need to be expanded to include attitude dynamics as well!
% Kyle is working on this now cuz Stew didn't wanna do it over the summer.
%
% We also need to nail down how we are representing time in order to enable all
% of our coordinate conversions to be complete. This will really unblock
% progress on the simulation.
function dy = j4(~, y)
    [gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
    dy = zeros(6, 1);
    dy(1:3, 1) = y(4:6, 1);
    dy(4:6, 1) = [gx; gy; gz];
end
