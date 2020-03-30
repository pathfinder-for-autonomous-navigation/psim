function [v_ecef] = velecef(self)
%VELECEF Returns the velocity of the orbit
%grav calls: 0
%args:
%   self(not propagating valid Orbit): The Orbit. 
%   v_ecef(3,1 matrix): velocity of the center of mass of the sat (m/s)
v_ecef=nan(3,1);
end

