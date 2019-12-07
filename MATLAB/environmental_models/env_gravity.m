function [acceleration,potential,hessian]= env_gravity(time,x)
%gravity Return the gravity acceleration vector, potential, and hessian in ECEF, units m/s^2
%   time(double): seconds since const.INITGPS_WN
%   x([3, 1]  vector): position in ECEF coords, units meters
%uses equation (3.154) in Fundamentals of Spacecraft Attitude Determination and Control
%   to calculate the hessian
%TODO add gravity from moon and sun and higher order spherical harmonics
global const
acceleration=zeros([3,1]);

out=geograv_wrapper(x);
acceleration(1:3)=out(1:3);
potential=out(4);
r=norm(x);
% potential= const.mu/r;%TODO get actual potential including spherical harmonics
% acceleration=zeros([3,1]);
% %[acceleration(1), acceleration(2), acceleration(3)] = gravityzonal(x', 'Earth', 4, 'Error');
% acceleration= -const.mu*x/r^3;
hessian= -const.mu/r^3*(eye(3)-3*(x*x')/r^2);
%from equation (3.154) in Fundamentals of Spacecraft Attitude Determination and Control
end





