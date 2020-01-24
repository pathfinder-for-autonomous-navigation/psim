function n = utl_orbrate(a)
% Calculates the orbital rate of the satellite assuming a circular orbit
% with the given semimajor axis.
%
%  a : semimajor axis (m)
%  n : orbital rate (rad / s)
global const

n = sqrt(const.mu ./ (a.^3));

end
