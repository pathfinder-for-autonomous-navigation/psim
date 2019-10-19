function [quat_ecef_eci,rate_ecef]= env_earth_attitude(time)
%earth_attitude Return the quaternion and anguar rate to rotate vectors from eci to ecef
%   time(double): time in seconds since const.INITGPS_WN
%#codegen
global const
rate_ecef=const.earth_rate_ecef;
quat_ecef0p_ecef0= [const.PRECESSION_RATE*time; 1.0;];
quat_ecef0p_ecef0= quat_ecef0p_ecef0/norm(quat_ecef0p_ecef0);
theta= const.earth_rate_ecef(3)*time;% earth rotation angle
quat_ecef_ecef0p= [0;0;sin(theta/2);cos(theta/2);];
quat_ecef0_eci=const.quat_ecef0_eci;
quat_ecef_eci=utl_quat_cross_mult(quat_ecef_ecef0p,utl_quat_cross_mult(quat_ecef0p_ecef0,quat_ecef0_eci));
end





