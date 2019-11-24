%----------------------- Begin Code Sequence -----------------------------%
% Purpose:                                                                %
% Convert WGS 84 (CTS, ECEF) Coordinates to ECI (CIS, Epoch J2000.0) Coordinates. This function has been vectorized for speed.               %
%                                                                         %
% Inputs:                                                                 %
%-------                                                                  %
%JD                     [1 x N]                         Julian Date Vector
%r_ECEF                 [3 x N]                         Position Vector in ECEF coordinate frame of reference
%v_ECEF                 [3 x N]                         Velocity Vector in ECEF coordinate frame of reference
%a_ECEF                 [3 x N]                         Acceleration Vector in ECEF coordinate frame of reference
% Outputs:
%---------                                                                
%r_ECI                  [3 x N]                         Position Vector in ECI coordinate frame of reference
%v_ECI                  [3 x N]                         Velocity vector in ECI coordinate frame of reference
%a_ECI                  [3 x N]                         Acceleration Vector in ECI coordinate frame of reference

% References:
%-------------
%Orbital Mechanics with Numerit, http://www.cdeagle.com/omnum/pdf/csystems.pdf
%
%
% Function Dependencies:
%------------------
% JD2GMST
%------------------------------------------------------------------       %
% Programed by Darin Koblick  07-17-2010                                  %
% Modified on 03/01/2012 to add acceleration vector support 
% Modified 12/12/2019 by Sruti
%------------------------------------------------------------------       %
function [r_ECI v_ECI] = ECEFtoECI(pantime,r_ECEF,v_ECEF)

[quat_ecef_eci,rate_ecef_eci_ecef]=env_earth_attitude(pantime);
quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
r_ECI=utl_rotateframe(quat_eci_ecef,r_ECEF);
v_ECEF= v_ECEF+ cross(rate_ecef_eci_ecef,r_ECEF);
v_ECI=utl_rotateframe(quat_eci_ecef,v_ECEF);

    
    