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



selfrv= rv_ecef;
targetrv= rv_ecef;
xest= [selfrv;targetrv;];
Sest= eye(12);
processvsdiv= 1E-8;
SQ= diag([zeros(3,1);processvsdiv*ones(3,1);zeros(3,1);processvsdiv*ones(3,1)]);
messdiv= [10*ones(6,1);0.5*ones(3,1);];
H= [eye(6), zeros(6);
    -eye(3),zeros(3),eye(3),zeros(3);];
n= 60*60*10;
samplerate=100;
global dt
dt=0.1;
t=0;
xests=zeros(12,n/samplerate);
xs=zeros(12,n/samplerate);
Sests=zeros(12,12,n/samplerate);
for i=1:n
    %update orbits
    selfrv=orbit_updater(selfrv,t,dt);
    targetrv=orbit_updater(targetrv,t,dt);
    t=t+dt;
    measure= cdgps_measurer(selfrv,targetrv,t);
    %update time 
    [J,xest]=jac_updaterfn(xest);
    [~, Sest]=sqrtDiscKalPred(xest,Sest,J,SQ);
    measurevalid= all(isfinite(measure)) && ...
          all(isreal(measure));   
    if measurevalid
        %correct estimate
        [xest, Sest,~,~,~]=sqrtKalmanUpdate(xest,Sest,measure,diag(messdiv),H);
    end
    
    if mod(i,samplerate)==0
        i
        xests(:,i/samplerate)=xest;
        xs(:,i/samplerate)=[selfrv;targetrv];
        Sests(:,:,i/samplerate)=Sest;
    end
    
end
    
function [J,x]=jac_updaterfn(x)
    global dt
    r= x(1:3);
    v= x(4:6);
    relr= x(7:9)-x(1:3);
    relv= x(10:12)-x(4:6);
    [r,v,Js,relr,relv,Jt] = orb_short_orbit_prop(r,v,relr,relv,dt,0);
    x= [r;v;r+relr;v+relv;];
    J= [Js,zeros(6);
        zeros(6), Jt;];
end    
    
function measure= cdgps_measurer(selfrv,targetrv,t)
%MEASURER single cdgps measurement
    gpspossdiv=5;
    cdgpspossdiv=0.25;
    gpsvelsdiv=5;
    measure=nan(9,1);
    if (mod(t,90*60)<90*60/2)
        measure(1:6)= selfrv + [gpspossdiv*randn(3,1)+5;gpsvelsdiv*randn(3,1)+5];
        measure(7:9)= targetrv(1:3)-selfrv(1:3)+cdgpspossdiv*randn(3,1)+0.25;
    end
end