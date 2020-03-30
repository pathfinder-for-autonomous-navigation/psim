function [self,jac] = shortupdate(self,dt_ns,earth_rate_ecef)
%SHORTUPDATE Does a short update of the orbit.
%grav calls: 1
%args:
%   self(not propagating valid Orbit): The Orbit. 
%   dt_ns(int64 in the range [-2E8,2E8]): Time step (ns)
%   earth_rate_ecef(3,1 matrix): Angular rate of the earth in ECEF
%   jac(6,6 matrix): 
%       The jacobian of y=f(x) where x and y are vectors
%           [r;
%            v;]
%       and f is the shortupdate function.


%only calc jacobian if requested, 
% in C++ this can be done with overloading
if nargout==2
    calc_jac=true;
    jac=nan(6,6);
else
    calc_jac=false;
end
end

