function eclipse = env_eclipse(earth2sat,sat2sun)
%env_eclipse returns true if the satellite is in eclipse
%   Assumes the sunlight is completely parallel.
%   earth2sat ([3, 1]  vector): Vector from earth to satellite (m)
%   sat2sun ([3, 1]  vector): unit vector from satellite to sun
global const
d=dot(earth2sat,sat2sun);
c_squared=dot(earth2sat,earth2sat);
b_squared=d*d;
a_squared=c_squared-b_squared;
if (d>=0)%above the light half of earth
   eclipse= false;
elseif (a_squared>(const.R_EARTH^2))%above earths shadow
   eclipse= false;
else
   eclipse= true;
end
end

