function n = utl_orbrate(a)
% Calculates the orbital rate of the sattelite assuming a circular orbit
% with the given semimajor axis.
global const

n = sqrt(const.mu / (a^3));

end
