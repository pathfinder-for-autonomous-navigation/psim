function [r_ECI,v_ECI] = env_ECEFtoECI(time,r_ECEF,v_ECEF)
%Returns a position and velocity in ECI from ECEF, units s, m, m/s
%  The shape of the vectors should be (3,1)
%   time(double): seconds since const.INITGPS_WN
%   Modified 12/12/2019 by Sruti
[quat_ecef_eci,rate_ecef_eci_ecef]=env_earth_attitude(time);
quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
r_ECI=utl_rotateframe(quat_eci_ecef,r_ECEF);
v_ECEF= v_ECEF+ cross(rate_ecef_eci_ecef,r_ECEF);
v_ECI=utl_rotateframe(quat_eci_ecef,v_ECEF);

    
    