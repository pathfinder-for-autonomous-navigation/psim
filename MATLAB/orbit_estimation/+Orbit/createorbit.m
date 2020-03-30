function [self] = createorbit(ns_gps_time,r_ecef,v_ecef)
%CREATEORBIT Creates an orbit from time, position, and velocity
%   Orbit self may be invalid if the inputs are bad. 
%   TODO describe bad inputs
%grav calls: 1
%args:
%   self(not propagating Orbit): The Orbit. 
%   ns_gps_time(int64): time since gps epoch (ns)
%   r_ecef (3,1 matrix): position of the center of mass of the sat (m)
%   v_ecef (3,1 matrix): velocity of the sat (m/s)
self=init();
end

