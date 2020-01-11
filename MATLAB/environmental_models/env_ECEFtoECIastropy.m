function [r_ECI,v_ECI] = env_ECEFtoECIastropy(time,r_ECEF,v_ECEF)
%Returns a position and velocity in ECI from ECEF, units s, m, m/s
%  The shape of the vectors should be (3,1)
%   time(double): seconds since const.INITGPS_WN
global const
a=cell(py.environmental_models.helper_functions.eci_ecef_conversions.ecef2eci(time,r_ECEF,v_ECEF,const.INITGPS_WN));
r_ECI= double(a{1})';
v_ECI= double(a{2})';


    
    