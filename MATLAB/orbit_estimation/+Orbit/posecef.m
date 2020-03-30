function [r_ecef] = posecef(self)
%POSECEF Returns the position of the orbit in ecef
%grav calls: 0
%args:
%   self(not propagating valid Orbit): The Orbit. 
%   r_ecef(3,1 matrix): position of the center of mass of the sat (m)
r_ecef= nan(3,1);
end

