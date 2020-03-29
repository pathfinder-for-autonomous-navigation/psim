function [isvalid] = valid(self)
%VALID Returns true if the Orbit self is valid
%   A valid orbit has finite and real position and velocity, is in low
%   earth orbit, and has a reasonable time stamp (within 20 years of the
%   initial gps week).
isvalid = false;
end

