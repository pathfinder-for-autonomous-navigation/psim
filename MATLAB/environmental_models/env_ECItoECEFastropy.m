function [r_ECEF,v_ECEF] = env_ECItoECEFastropy(time,r_ECI,v_ECI)
%Returns a position and velocity in ECEF from ECI, units s, m, m/s
%  The shape of the vectors should be (3,1)
%   time(double): seconds since const.INITGPS_WN
global const
a=cell(py.environmental_models.helper_functions.eci_ecef_conversions.eci2ecef(time,r_ECI,v_ECI,const.INITGPS_WN));
r_ECEF= double(a{1})';
v_ECEF= double(a{2})';


    
    