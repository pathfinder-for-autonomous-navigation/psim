global const
config();
a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.0;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0*pi/180;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e*e), e, i, O, o, nu, const.mu);

r2 = r1;
v2 = randn(3, 1);
v2= cross(v1,v2);
V_rel   = 2.0; % Relative velocity at deployment (m/s)
v2 = v1 + 0*V_rel * (v2 / norm(v2))+ -1*V_rel * (v1/norm(v1));
t_drift=1.5*60*60/10;
N=2000*10;
ang_moment1= zeros(3,N);
ang_moment2= zeros(3,N);
for i= 1:N
    i
    [t, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift);
    ang_moment1(:,i)=cross(r1,v1);
    ang_moment2(:,i)=cross(r2,v2);
end

%average over ang moment over orbit
numavg= 10;
avg_ang_moment1= zeros(3,floor(N/numavg));
avg_ang_moment2= zeros(3,floor(N/numavg));
for i= 1:floor(N/numavg)
    avg_ang_moment1(:,i)=sum(ang_moment1(:,(i-1)*numavg+1:i*numavg),2)/numavg;
    avg_ang_moment2(:,i)=sum(ang_moment2(:,(i-1)*numavg+1:i*numavg),2)/numavg;
end


figure;
hold on
plot(1:floor(N/numavg),avg_ang_moment1(1,:),'-r')
plot(1:floor(N/numavg),avg_ang_moment2(1,:),'-b')
title('x1 eci angular momentum')
xlabel('Time (s)')
ylabel('angular momentum (m^2/s)')
figure;
plot(1:floor(N/numavg),avg_ang_moment1(2,:))
title('y1-y2 eci angular momentum drift')
xlabel('Time (s)')
ylabel('angular momentum (m^2/s)')
figure;
plot(1:floor(N/numavg),avg_ang_moment1(3,:))
title('z1-z2 eci angular momentum drift')
xlabel('Time (s)')
ylabel('angular momentum (m^2/s)')
