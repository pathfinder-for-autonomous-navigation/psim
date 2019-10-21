function F_envdrag = env_atmospheric_drag(time,r,v)
%input: mission time in sec
% r position in ECI frame in m
% v velocity in ECI frame in m/s
% Omega longitude/right ascension of the ascending node in rad

% outputs: Fenv_drag in N in direction of velocity % check dis ECI frame? 
% assumes a simple, fully static exponentially decaying model (Vallado)

global const
%time conversions
UTC_leap = utl_time2datetime(time,const.INITGPS_WN);  %converts mission time in secs to datetime in UTCleapseconds
%UTC_leap = utl_time2datetime(time,3); test
offset = 2*86400; %offset of UTCLeapSeconds to UTC time
del = datetime(0,0,0,0,0,offset);
del.TimeZone = 'UTCLeapSeconds';
UTC = UTC_leap - offset;

% r: satellite position in ECI
%convert ECI coordinates to latitude, longitude, altitude (LLA) geodetic coordinates based on the Universal Coordinated Time (UTC) you specify
lla = eci2lla(r, [UTC.Year UTC.Month UTC.Day UTC.Hour UTC.Minute UTC.Second]); 

h = abs(lla(3)); %altitude of Position (m); geocentric altitude = geodetic altitude

rho = get_rho(h);

Cd = 0.8; %drag coefficient of angled cube
A = 0.1*sqrt(2)*0.3; %largest planar area of satellite in m^2
v_rel = v - cross(const.earth_rate_ecef,r); %velocity relative to the rotating atmosphere
%v_rel = v - cross([0;0;7.2921158553E-5],r); test

F_envdrag = -0.5*rho*Cd*A*(v_rel*v_rel'); %drag calculated in eci frame

end

function rho = get_rho(h)
    
    % in meters
    hmin = 1E3*[0;25;30;35;40;45;50;55;60;65;...
        70;75;80;85;90;95;100;110;120;130;...
        140;150;160;180;200;250;300;350;400;...
        450;500;600;700;800;900;1000];
    % in meters
    hmax = 1E3*[25;30;35;40;45;50;55;60;65;70;75;...
        80;85;90;95;100;110;120;130;140;150;160;...
        180;200;250;300;350;400;450;500;600;...
        700;800;900;1000;inf];
    % in meters
    h0 = 1E3*[0;25;30;35;40;45;50;55;60;...
        65;70;75;80;85;90;95;100;110;...
        120;130;140;150;160;180;200;250;300;...
        350;400;450;500;600;700;800;900;1000];
    % kg/m^3
    rho0 = [1.225; 3.899E-2; 1.774E-2;...
        8.279E-3; 3.972E-3; 1.995E-3;...
        1.057E-3; 5.821E-4; 3.206E-4; ...
        1.718E-4; 8.770E-5; 4.178E-5;...
        1.905E-5; 8.337E-6; 3.396E-6; ...
        1.343E-6; 5.297E-7; 9.661E-8; ...
        2.438E-8; 8.484E-9; 3.845E-9; ...
        2.070E-9; 1.224E-9; 5.464E-10;...
        2.789E-10; 7.248E-11; 2.418E-11;...
        9.158E-12; 3.725E-12; 1.585E-12;...
        6.967E-13; 1.454E-13; 3.614E-14;...
        1.170E-14; 5.245E-15; 3.019E-15];
    % in meters
    H = 1E3*[8.44; 6.49; 6.75; 7.07; 7.47; 7.83;...
        7.95; 7.73; 7.29; 6.81; 6.33; 6.00;...
        5.70; 5.41; 5.38; 5.74; 6.15; 8.06;...
        11.6; 16.1; 20.6; 24.6; 26.3; 33.2;...
        38.5; 46.9; 52.5; 56.4; 59.4; 62.2;...
        65.8; 79.0; 109.0; 164.0; 225.0; 268.0];
    
    idx = 0;
    for i = 1:length(hmin)
        if abs(h) >= hmin(i) && abs(h) < hmax(i)
            idx = i;
            break
        end
    end
    
    rho = rho0(idx)*exp(-(h-h0(idx))/H(idx));
    
    
end

    

    