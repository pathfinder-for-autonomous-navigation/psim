function [isvalid] = valid(self)
%VALID Returns true if the Orbit self is valid
%   A valid orbit has finite and real position and velocity, is in low
%   earth orbit, and has a reasonable time stamp (within 20 years of pan epoch).
%   The validity check should not reject 
%   gps readings due to reasonable noise of:
%       TODO add max expected gps error
%
%   Low earth orbit is a defined:
%           Eccentricity(TODO add range)
%           Semimajor axis (TODO add range)
%grav calls: 0
%args:
%   self(Orbit): The Orbit. 
isvalid = false;
end

