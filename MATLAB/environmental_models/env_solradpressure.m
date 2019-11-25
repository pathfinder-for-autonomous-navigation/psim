function a = env_solradpressure(r,r_Sun,mass)

%References:
% Montenbruck O., Gill E.; Satellite Orbits: Models, Methods and 
% Applications; Springer Verlag, Heidelberg; Corrected 3rd Printing (2005).
%
% Montenbruck O., Pfleger T.; Astronomy on the Personal Computer; Springer 
% Verlag, Heidelberg; 4th edition (2000).
%
% Seeber G.; Satellite Geodesy; Walter de Gruyter, Berlin, New York; 2nd
% completely revised and extended edition (2003).
%
% Vallado D. A; Fundamentals of Astrodynamics and Applications; McGraw-Hill
% , New York; 3rd edition(2007).
%
% http://sol.spacenvironment.net/jb2008/
%
% http://ssd.jpl.nasa.gov/?ephemerides

%  Computes the acceleration due to solar radiation pressure assuming the spacecraft surface normal to the Sun direction
%  assumes a cylindrical shadow model
%  Inputs:
%   r           Spacecraft position vector in m
%   r_Sun       Sun position vector (geocentric) in m
%   Area        Cross-section in m^2
%   mass        Spacecraft mass in kg

% Output:
%   a    		Acceleration (a=d^2r/dt^2)
%
% Notes:
%   r, r_sun, Area, mass, P0 and AU must be given in consistent units,
%   e.g. m, m^2, kg and N/m^2. 


% nu     Illumination factor:
% nu=0   Spacecraft in Earth shadow 
% nu=1   Spacecraft fully illuminated by the Sun

global const 

Fs = 1367; %solar constant
c = 3E8; %speed of light
A = const.satArea;
q = 1; %conservative
i = 0; %conservative

Fmag = 57* (Fs/c)*A*(1+q)*cos(i); %scaled up for GRACE


% Cr = const.Cr;        %Solar radiation pressure coefficient
% P_Sol = const.P_Sol;      %Solar radiation pressure at 1 AU in N/m^2
% AU = const.AU;         %Length of one Astronomical Unit
%     
% e_Sun = r_Sun / norm(r_Sun);   % Sun direction unit vector
% s     = dot ( r, e_Sun );      % Projection of s/c position 
% 
% if ( s>0 || norm(r-s*e_Sun)>const.R_EARTH )
%     nu = 1;
% else
%     nu = 0;
% end

% acceleration 
a = Fmag*r/(norm(r)^3); %acceleration in ECI
end



