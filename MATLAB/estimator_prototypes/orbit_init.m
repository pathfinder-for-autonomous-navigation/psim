function rv_ecef=orbit_init()
% Returns a random orbit, a 6x1 matrix of r (m) and v (m/s) in ECEF
global const

a  = 6860636.6;  % Semimajor axis                        (m)
e  = 0.001;      % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)

[   r,...  % Position (m)   [eci]
    v,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a*(1-e*e), e, i, O, o, nu, const.mu);
[r_ECEF,v_ECEF] = env_ECItoECEF(0,r,v);
rv_ecef= [r_ECEF;v_ECEF];

