%--------------------------------------------------------------------------
%
%               High Precision Orbit Propagator
%
% References:
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
%
% Last modified:   2018/02/11   M. Mahooti
%
%--------------------------------------------------------------------------

clc
clear all
format long g
close all
profile on
tic

global const Cnm Snm AuxParam eopdata SOLdata DTCdata APdata PC

SAT_Const
constants
load DE430Coeff.mat
PC = DE430Coeff;

% read Earth gravity field coefficients
Cnm = zeros(181,181);
Snm = zeros(181,181);
fid = fopen('GGM03S.txt','r');
for n=0:180
    for m=0:n
        temp = fscanf(fid,'%d %d %f %f %f %f',[6 1]);
        Cnm(n+1,m+1) = temp(3);
        Snm(n+1,m+1) = temp(4);
    end
end
fclose(fid);

% read Earth orientation parameters
fid = fopen('eop19990101.txt','r');
%  ----------------------------------------------------------------------------------------------------
% |  Date    MJD      x         y       UT1-UTC      LOD       dPsi    dEpsilon     dX        dY    DAT
% |(0h UTC)           "         "          s          s          "        "          "         "     s 
%  ----------------------------------------------------------------------------------------------------
eopdata = fscanf(fid,'%i %d %d %i %f %f %f %f %f %f %f %f %i',[13 inf]);
fclose(fid);

% read solar storm indices
fid = fopen('SOLFSMY.txt','r');
%  ------------------------------------------------------------------------
% | YYYY DDD   JulianDay  F10   F81c  S10   S81c  M10   M81c  Y10   Y81c
%  ------------------------------------------------------------------------
SOLdata = fscanf(fid,'%d %d %f %f %f %f %f %f %f %f %f',[11 inf]);
fclose(fid);

% read geomagnetic storm indices
fid = fopen('DTCFILE.txt','r');
%  ------------------------------------------------------------------------
% | DTC YYYY DDD   DTC1 to DTC24
%  ------------------------------------------------------------------------
DTCdata = fscanf(fid,'%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d',[26 inf]);
fclose(fid);

% read geomagnetic storm indices
fid = fopen('SOLRESAP.txt','r');
%  ------------------------------------------------------------------------
% | YYYY DDD  F10 F10B Ap1 to Ap8
%  ------------------------------------------------------------------------
APdata = fscanf(fid,'%d %d %f %f %f %f %f %f %f %f %f',[12 inf]);
fclose(fid);

% model parameters
AuxParam = struct('Mjd_UTC',0,'area_solar',0,'area_drag',0,'mass',0,'Cr',0,...
                  'Cd',0,'n',0,'m',0,'sun',0,'moon',0,'sRad',0,'drag',0,...
                  'planets',0,'SolidEarthTides',0,'OceanTides',0,'Relativity',0);

% epoch state (Envisat)
fid = fopen('InitialState.txt','r');
tline = fgetl(fid);
year = str2num(tline(1:4));
mon = str2num(tline(6:7));
day = str2num(tline(9:10));
hour = str2num(tline(12:13));
min = str2num(tline(15:16));
sec = str2num(tline(18:23));
tline = fgetl(fid);
Y0(1) = str2num(tline);
tline = fgetl(fid);
Y0(2) = str2num(tline);
tline = fgetl(fid);
Y0(3) = str2num(tline);
tline = fgetl(fid);
Y0(4) = str2num(tline);
tline = fgetl(fid);
Y0(5) = str2num(tline);
tline = fgetl(fid);
Y0(6) = str2num(tline);
tline = fgetl(fid);
AuxParam.area_solar = str2num(tline(49:end));
tline = fgetl(fid);
AuxParam.area_drag = str2num(tline(38:end));
tline = fgetl(fid);
AuxParam.mass = str2num(tline(19:end));
tline = fgetl(fid);
AuxParam.Cr = str2num(tline(5:end));
tline = fgetl(fid);
AuxParam.Cd = str2num(tline(5:end));
fclose(fid);

% epoch
Mjd_UTC = Mjday(year, mon, day, hour, min, sec);
Y0 = ECEF2ECI(Mjd_UTC, Y0);

AuxParam.Mjd_UTC = Mjd_UTC;
AuxParam.n       = 40;
AuxParam.m       = 40;
AuxParam.sun     = 1;
AuxParam.moon    = 1;
AuxParam.planets = 1;
AuxParam.sRad    = 1;
AuxParam.drag    = 1;
AuxParam.SolidEarthTides = 1;
AuxParam.OceanTides = 0;
AuxParam.Relativity = 0;

Mjd0   = Mjd_UTC;
Step   = 60;   % [s]
N_Step = 1588; % 26.47 hours

% propagation
Eph = Ephemeris(Y0, N_Step, Step);

fid = fopen('SatelliteStates.txt','w');
for i=1:N_Step+1
    [year,mon,day,hr,min,sec] = invjday(Mjd0+Eph(i,1)/86400+2400000.5);
    fprintf(fid,'  %4d/%2.2d/%2.2d  %2d:%2d:%6.3f',year,mon,day,hr,min,sec);
    fprintf(fid,'  %14.3f%14.3f%14.3f%12.3f%12.3f%12.3f\n',...
            Eph(i,2),Eph(i,3),Eph(i,4),Eph(i,5),Eph(i,6),Eph(i,7));
end
fclose(fid);

[n, m] = size(Eph);
Eph_ecef = zeros(n,m);
for i=1:n
    Eph_ecef(i,1) = Eph(i,1);
    Eph_ecef(i,2:7) = ECI2ECEF(Mjd0+Eph_ecef(i,1)/86400, Eph(i,2:7));    
end

True_EnvisatStates
dd = True_Eph-Eph_ecef(:,2:7);

% Plot orbit in ECI reference
figure(1)
plot3(Eph(:,2),Eph(:,3),Eph(:,4),'o-r')
grid;
title('Orbit ECI (inertial) (m)')

% Plot orbit in ECEF reference
figure(2)
plot3(Eph_ecef(:,2),Eph_ecef(:,3),Eph_ecef(:,4),'-')
title('Orbit ECEF (m)')
xlabel('X');ylabel('Y');zlabel('Z');
grid

% Plot Discrepancy of Precise and Propagated orbits
figure(3)
subplot(3,1,1);
plot(dd(:,1));
title('Discrepancy of Precise and Propagated Envisat Positions for 26.47 hours');
axis tight
xlabel('Time')
ylabel('dX[m]')
hold on
subplot(3,1,2);
plot(dd(:,2));
axis tight
xlabel('Time')
ylabel('dY[m]')
subplot(3,1,3);
plot(dd(:,3));
axis tight
xlabel('Time')
ylabel('dZ[m]')
toc

profile viewer
profile off



function dY = Accel(t, Y)

    %--------------------------------------------------------------------------
    % Accel: Computes the acceleration of an Earth orbiting satellite due to 
    %    	 - solar radiation pressure
    %
    % Inputs:
    %   Mjd_UTC     Modified Julian Date (UTC)
    %   Y           Satellite state vector in the ICRF/EME2000 system
    %   Area        Cross-section 
    %   mass        Spacecraft mass
    %   Cr          Radiation pressure coefficient
    %   Cd          Drag coefficient
    %
    % Output:
    %   dY          Acceleration (a=d^2r/dt^2) in the ICRF/EME2000 system
    % 
    %--------------------------------------------------------------------------

    global const AuxParam eopdata

    MJD_UTC = AuxParam.Mjd_UTC+t/86400;
    [x_pole,y_pole,UT1_UTC,LOD,dpsi,deps,dx_pole,dy_pole,TAI_UTC] = IERS(eopdata,MJD_UTC,'l');
    [UT1_TAI,UTC_GPS,UT1_GPS,TT_UTC,GPS_UTC] = timediff(UT1_UTC,TAI_UTC);

    JD = MJD_UTC+2400000.5;
    [year, month, day, hour, minute, sec] = invjday(JD);
    [DJMJD0, DATE] = iauCal2jd(year, month, day);
    TIME = (60*(60*hour + minute) + sec)/86400;
    UTC = DATE + TIME;
    TT = UTC + TT_UTC/86400;
    TUT = TIME + UT1_UTC/86400;
    UT1 = DATE + TUT;

    % Polar motion matrix (TIRS->ITRS, IERS 2003)
    Pi = iauPom00(x_pole, y_pole, iauSp00(DJMJD0, TT));
    % Form bias-precession-nutation matrix
    NPB = iauPnm06a(DJMJD0, TT);
    % Form Earth rotation matrix
    gast = iauGst06(DJMJD0, UT1, DJMJD0, TT, NPB);
    Theta  = iauRz(gast, eye(3));
    % ICRS to ITRS transformation
    E = Pi*Theta*NPB;

    MJD_TDB = Mjday_TDB(TT);
    [r_Mercury,r_Venus,r_Earth,r_Mars,r_Jupiter,r_Saturn,r_Uranus, ...
     r_Neptune,r_Pluto,r_Moon,r_Sun,r_SunSSB] = JPL_Eph_DE430(MJD_TDB);

    % Acceleration due to harmonic gravity field
    a = AccelHarmonic_ElasticEarth(MJD_UTC,r_Sun,r_Moon,Y(1:3),E,UT1_UTC,TT_UTC,x_pole,y_pole);
    % a = AccelHarmonic_AnelasticEarth(MJD_UTC,r_Sun,r_Moon,Y(1:3),E,UT1_UTC,TT_UTC,x_pole,y_pole);

    % Luni-solar perturbations
    if (AuxParam.sun)
        a = a + AccelPointMass(Y(1:3),r_Sun,const.GM_Sun);
    end

    if (AuxParam.moon)
        a = a + AccelPointMass(Y(1:3),r_Moon,const.GM_Moon);
    end

    % Planetary perturbations
    if (AuxParam.planets)
        a = a + AccelPointMass(Y(1:3),r_Mercury,const.GM_Mercury);
        a = a + AccelPointMass(Y(1:3),r_Venus,const.GM_Venus);
        a = a + AccelPointMass(Y(1:3),r_Mars,const.GM_Mars);
        a = a + AccelPointMass(Y(1:3),r_Jupiter,const.GM_Jupiter);
        a = a + AccelPointMass(Y(1:3),r_Saturn,const.GM_Saturn);
        a = a + AccelPointMass(Y(1:3),r_Uranus,const.GM_Uranus);    
        a = a + AccelPointMass(Y(1:3),r_Neptune,const.GM_Neptune);
        a = a + AccelPointMass(Y(1:3),r_Pluto,const.GM_Pluto);
    end

    % Solar radiation pressure
    if (AuxParam.sRad)
        a = a + AccelSolrad(Y(1:3),r_Earth,r_Moon,r_Sun,r_SunSSB, ...
            AuxParam.area_solar,AuxParam.mass,AuxParam.Cr,const.P_Sol,const.AU,'geometrical');
    end

    % Atmospheric drag
    if (AuxParam.drag)
        % Atmospheric density
        % Omega = 7292115.8553e-11+4.3e-15*( (MJD_UTC-const.MJD_J2000)/36525 ); % [rad/s]
        Omega = const.omega_Earth-0.843994809*1e-9*LOD; % IERS [rad/s]
        [~,dens] = JB2008(MJD_UTC,r_Sun,Y(1:3));
        a = a + AccelDrag(dens,Y(1:3),Y(4:6),NPB,AuxParam.area_drag,AuxParam.mass,AuxParam.Cd,Omega);
    end

    % Relativistic Effects
    if (AuxParam.Relativity)
        a = a + Relativity(Y(1:3),Y(4:6));
    end

end

function a = AccelSolrad(r,r_Earth,r_Moon,r_Sun,r_SunSSB,Area,mass,Cr,P0,AU,shm)

%--------------------------------------------------------------------------
%
% AccelSolrad: Computes the acceleration due to solar radiation pressure
%			   assuming the spacecraft surface normal to the Sun direction
%
% Inputs:
%   r           Spacecraft position vector 
%   r_Earth	    Earth position vector (Barycentric)
%   r_Moon		Moon position vector (geocentric)
%   r_Sun       Sun position vector (geocentric)
%   r_SunSSB    Sun position vector (Barycentric)
%   Area        Cross-section 
%   mass        Spacecraft mass
%   Cr          Solar radiation pressure coefficient
%   P0          Solar radiation pressure at 1 AU 
%   AU          Length of one Astronomical Unit
%   shm         Shadow model (geometrical or cylindrical)
%
% Output:
%   a    		Acceleration (a=d^2r/dt^2)
%
% Notes:
%   r, r_sun, Area, mass, P0 and AU must be given in consistent units,
%   e.g. m, m^2, kg and N/m^2. 
%
% Last modified:   2018/01/27   M. Mahooti
% 
%--------------------------------------------------------------------------

%        Moon wrt Earth          pccor      rpc
%        Earth wrt Sun           ccor       rc
%        Moon wrt Sun            pscor      rps   
%        Satellite wrt Earth     sbcor      rsb  
%        Satellite wrt Sun       bcor       rb 
%        Satellite wrt Moon      sbpcor     rsbp
pccor = r_Moon;
ccor = r_Earth-r_SunSSB;
pscor = r_Moon-r_Sun;
sbcor = r;
bcor = r-r_Sun;
sbpcor = r-r_Moon;

if ( strcmp(shm,'cylindrical') )
    nu = Cylindrical(r,r_Sun);
else
    [nu,~] = Shadow(pccor,ccor,pscor,sbcor,bcor,sbpcor);
end

% Acceleration
a = nu*Cr*(Area/mass)*P0*(AU*AU)*bcor/(norm(bcor)^3);

end

function nu = Cylindrical(r, r_Sun)
    %--------------------------------------------------------------------------
    % 
    % Cylindrical: Computes the fractional illumination of a spacecraft in the 
    %              vicinity of the Earth assuming a cylindrical shadow model
    % 
    % Inputs:
    %   r         Spacecraft position vector [m]
    %   r_Sun     Sun position vector [m]
    %
    % Output:
    %   nu        Illumination factor:
    %             nu=0   Spacecraft in Earth shadow 
    %             nu=1   Spacecraft fully illuminated by the Sun
    %
    % Last modified:   2018/01/27   M. Mahooti
    %
    %--------------------------------------------------------------------------

    global const

    e_Sun = r_Sun / norm(r_Sun);   % Sun direction unit vector
    s     = dot ( r, e_Sun );      % Projection of s/c position 

    if ( s>0 || norm(r-s*e_Sun)>const.R_Earth )
        nu = 1;
    else
        nu = 0;
    end
end


function [lambda,ecltyp] = Shadow(pccor,ccor,pscor,sbcor,bcor,sbpcor)

    %--------------------------------------------------------------------------
    %    Computation of fraction (lambda) of solar disk seen by spacecraft
    %           Beebe, King, Reasonberg, Preston   June 19971
    % 
    %                                Vector    Distance
    %        Moon wrt Earth          pccor      rpc
    %        Earth wrt Sun           ccor       rc
    %        Moon wrt Sun            pscor      rps   
    %        Satellite wrt Earth     sbcor      rsb  
    %        Satellite wrt Sun       bcor       rb 
    %        Satellite wrt Moon      sbpcor     rsbp
    %
    % Last modified:   2018/01/27   M. Mahooti
    %
    %--------------------------------------------------------------------------
    global const

    % shadow computation - geometric model
    % lambda = 1 - no shadow
    % lambda = 0 - no sunlight
    % 0 < lambda < 1 - partial shadow

    % no consideration is given to the change of direction associated
    % with partial shadow.
    lambda=1;
    ecltyp = char(1);

    % Check for both eclipses of the Sun by both the Earth and the Moon
    % First the Earth
    ubcor = zeros(3,1);
    rb = norm(bcor);
    rc = norm(ccor);

    if(rb<=rc)
    else
        % get the unit vector of the satellite relative to the sun
        for i=1:3
            ubcor(i)=bcor(i)/rb;
        end

        sepp = cross(sbcor,ubcor);

        % rsbx is the projection of sbcor along bcor
        rsbx = dot(sbcor,ubcor);

        % rs, rp are apparent (from satellite) radii of sun and earth
        % sep is apparent separation of their centers
        rs=const.R_Sun/rb;
        rp=const.R_Earth/rsbx;
        sep=sqrt(sepp(1)^2+sepp(2)^2+sepp(3)^2)/rsbx;

        lambda = get_lambda(rs,rp,sep);
    end
    % If no Earth eclipse, check the Moon
    if(lambda<1)
        ecltyp = 'E';
        return
    else

        for i=1:3
            pscor(i) = pccor(i) + ccor(i);
            sbpcor(i) = sbcor(i) - pccor(i);
        end

        rps = sqrt(pscor(1)^2+pscor(2)^2+pscor(3)^2);

        if(rb<=rps)
            return
        end

        %   unit vector of SV wrt Sun
        for i=1:3
            ubcor(i)=bcor(i)/rb;
        end

        % rsbx is the projection of sbcor along bcor
        rsbx=dot(sbpcor,ubcor);

        % rs, rp are apparent (from satellite) radii of sun and moon
        % sep is apparent angular separation of their centers
        rs=const.R_Sun/rb;
        rp=const.R_Moon/rsbx;

        sepp=cross(sbpcor,ubcor);
        sep=sqrt(sepp(1)^2+sepp(2)^2+sepp(3)^2)/rsbx;

        lambda = get_lambda(rs,rp,sep);

        if( lambda<1 )
            ecltyp = 'M';
        end    
    end
end

function lambda = get_lambda(rs,rp,sep)

    % Calculate lambda
    % 
    % rs : apparent radius of sun as viewed from satellite (radians)
    % rp : apparent radius of eclipsing body as viewed from satellite (radians)
    % sep: apparent separation of the center of the Sun and eclipsing body (radians)
    % 
    % lambda : fraction of Sun's disk visible (1.0 = no eclipse; 0 = total eclipse)

    if (rs+rp<=sep)
        % no eclipse
        lambda = 1;
        return
    elseif( rp-rs>=sep )
        % full eclipse
        lambda = 0;
        return
    else
        % partial eclipse, do the calculations
        if(sep<=rs-rp)
        else
            % set r1 = smaller disc, r2 = larger
            if(rs>rp)
                r1=rp;
                r2=rs;
            else
                r1=rs;
                r2=rp;        
            end        
            % phi = 1/2 angle subtended in disc 1 by arc of intersection
            phi = acos((r1*r1+sep*sep-r2*r2)/(2*r1*sep));
            if (phi<0)
                phi = pi + phi;
            end
            if(r2/r1>5)
                hgt=sqrt(r1^2-(sep-r2)^2);
                area2=hgt*(sep-r2);
                area3=0;
            else
                % thet = 1/2 angle subtended in disc 2 by arc of intersection
                % hgt  = 1/2 linear distance between ends of arc of intersection
                hgt=r1*sin(phi);
                thet=asin(hgt/r2);
                area2=sep*hgt;
                area3=thet*r2^2;            
            end
            % one disc much bigger - treat boundary as a straight line        
            area1=(pi-phi)*r1^2;
            % ari = area of non-overlapped portion of small disc
            ari=area1+area2-area3;
            area1=pi*rs^2;
            if(rs>rp)
                area2=pi*rp^2;
                lambda=(area1+ari-area2)/area1;
                return
            else
                % sun is small disc
                lambda=ari/area1;
                return
            end
            % eclipsing body is small disc
        end
        % eclipsing body lies within sun's disc - what fraction of sun's disk is blocked
        lambda=(rs^2-rp^2)/rs^2;   
    end
end
