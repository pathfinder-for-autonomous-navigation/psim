function [r_ECEF,v_ECEF] = env_ECItoECEF(time,r_ECI,v_ECI)
%Returns a position and velocity in ECEF from ECI, units s, m, m/s
%  The shape of the vectors should be (3,1)
%   time(double): seconds since const.INITGPS_WN
%   Modified 12/12/2019 by Sruti
%   Modified 29 Jan 2020 by Nathan Zimmerberg (nhz2)
[quat_ecef_eci,rate_ecef_eci_ecef]=env_earth_attitude(time);
r_ECEF= utl_rotateframe(quat_ecef_eci,r_ECI);
v_ECEF= utl_rotateframe(quat_ecef_eci,v_ECI);
v_ECEF= v_ECEF - cross(rate_ecef_eci_ecef,r_ECEF);


    
    