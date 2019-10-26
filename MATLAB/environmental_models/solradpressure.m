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

AccelSolrad(Y(1:3),r_Earth,r_Moon,r_Sun,r_SunSSB, ...
            AuxParam.area_solar,AuxParam.mass,AuxParam.Cr,const.P_Sol,const.AU,'geometrical')

global const Cnm Snm AuxParam eopdata SOLdata DTCdata APdata PC

if (AuxParam.sRad)
    a = a + AccelSolrad(Y(1:3),r_Earth,r_Moon,r_Sun,r_SunSSB, ...
        AuxParam.area_solar,AuxParam.mass,AuxParam.Cr,const.P_Sol,const.AU,'geometrical');
end

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
