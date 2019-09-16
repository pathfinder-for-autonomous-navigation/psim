function S= env_sun_vector(time)
%sun_vector Return the normalized vector from earth to sun in ECI
%   time(double): seconds since const.INITGPS_WN
global const
S= planetEphemeris(juliandate(utl_time2datetime(time,const.INITGPS_WN)),'Earth','Sun');
S= (S/norm(S))';
end

