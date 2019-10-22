function B= env_magnetic_field(time,x)
%magnetic_field Return the magnetic feild vector in ECEF units Tesla
%   time(double): seconds since const.INITGPS_WN
%   x([3, 1]  vector): position in ECEF coords, units meters
%#codegen
global const
B=geomag_wrapper(const.INIT_DYEAR+time/365/24/60/60,x);
end





