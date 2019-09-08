function [quat_ecef_eci,rate_ecef]= earth_attitude(time)
%earth_attitude Return the quaternion and anguar rate to rotate vectors from eci to ecef
%   time(double): time in seconds since const.INITGPS_WN
global const
T=time2datetime(0,const.INITGPS_WN);
dcm=dcmeci2ecef_noerror('IAU-2000/2006',[year(T),month(T),day(T),hour(T),minute(T),second(T)+time]);
q=dcm2quat(dcm);
quat_ecef_eci(4)=q(1);
quat_ecef_eci(1)=q(2);
quat_ecef_eci(2)=q(3);
quat_ecef_eci(3)=q(4);
rate_ecef=[0;0;7.2921158553E-5;];
quat_ecef_eci=quat_ecef_eci';
%TODO properly handle higher accuracy terms, leap seconds, and rate calculation
end





