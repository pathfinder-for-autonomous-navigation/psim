function [self,num_grav_calls] = startpropagator(self,end_gps_time_ns,earth_rate_ecef)
%STARTPROPAGATOR Puts the orbit in propagator mode.
%   call finishpropagator or onegravcall numgravcallsleft times to finish the propagation.
%   This function can also be used to change the end time of a propagating
%   Orbit.
%grav calls: 0
%args:
%   self(valid Orbit): The Orbit. 
%   end_gps_time_ns(int64): Final gps time at the end of the propagation (ns)
%   earth_rate_ecef(3,1 matrix): Angular rate of the earth in ECEF
num_grav_calls=0;
end

