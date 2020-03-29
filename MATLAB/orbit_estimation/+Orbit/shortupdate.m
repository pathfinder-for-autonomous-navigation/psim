function [self,jac] = shortupdate(self,dt_ns,earth_rate_ecef)
%SHORTUPDATE Does a short update of the orbit with only one gav call.

%only calc jacobian if requested, 
% in C++ this can be done with overloading
if nargout==2
    calc_jac=true;
    jac=nan(6,6);
else
    calc_jac=false;
end
end

