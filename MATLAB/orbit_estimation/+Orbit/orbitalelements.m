function [elements] = orbitalelements(self)
%ORBITALELEMENTS returns a struct of the orbital elements
%grav calls: 0
%args:
%   self(not propagating valid Orbit): The Orbit. 
%   elements(struct with the following):
%       a: osculating semimajor axis (m)
%       e: osculating eccentricity (unitless)
%       i: osculating right ascension of the ascending node (rad)
%       O: osculating argument of perigee (rad)
%       o: osculating argument of perigee (rad)
%       nu: osculating true anomaly (rad)  
elements=struct();
end

