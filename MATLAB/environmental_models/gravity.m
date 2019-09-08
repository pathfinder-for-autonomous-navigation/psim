function g= gravity(time,x)
%gravity Return the gravity acceleration vector in ECEF, units m/s^2
%   time(double): seconds since const.INITGPS_WN
%   x([3, 1]  vector): position in ECEF coords, units meters
g=zeros([3,1]);
[g(1), g(2), g(3)] = gravitysphericalharmonic(x);
%TODO add gravity from moon and sun
end





