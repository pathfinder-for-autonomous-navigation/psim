function S= env_sun_vector(time)
%sun_vector Return the normalized vector from earth to sun in ECI
%   time(double): seconds since const.INITGPS_WN
global const

%find M in rads
M = 2*pi*(time - const.tp_earth)/const.period_earth;

% find E in rads using fixed point iteration see
% https://en.wikipedia.org/wiki/Kepler%27s_equation

E= M + const.e_earth*sin(M);

% for n= 0:100
%     E= M+e*sin(E);
% end

%get xses and yses (position in perifocal frame)
xses= cos(E) - const.e_earth;
yses= (1-0.5*const.e_earth*const.e_earth)*sin(E);
d= sqrt(xses*xses+yses*yses);
xses= xses/d; % [AU]
yses= yses/d; % [AU]
r_earth = [xses; yses; 0];

% rotate into inertial frame (ICRF or ECI)
S = -utl_rotateframe(const.quat_eci_perifocal,r_earth);

end

