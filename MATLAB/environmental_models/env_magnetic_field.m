function B= env_magnetic_field(time,x)
%magnetic_field Return the magnetic feild vector in ECEF units Tesla
%   time(double): seconds since const.INITGPS_WN
%   x([3, 1]  vector): position in ECEF coords, units meters
global const
B=zeros([3,1]);
x_lla= ecef2lla(x')';
[xyz,~,~,~,~]= wrldmagm_noerror(x_lla(3), x_lla(1), x_lla(2), decyear(utl_time2datetime(time,const.INITGPS_WN)), '2015');
%now B is in NED coords, and must be transformed back to ECEF.
[B(1),B(2),B(3)]= ned2ecefv(xyz(1),xyz(2),xyz(3),x_lla(1),x_lla(2));
B= 1E-9*B;
%TODO eventually this should use the 2020 model
end





