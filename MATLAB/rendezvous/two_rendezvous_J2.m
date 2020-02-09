clear all
global const
config();
M = 4.0; % Mass (kg)
J_min = 2e-4; % Min impulse (Ns)
J_max = 2e-2;%2e-2; % Max impulse (Ns)
max_dv_thrust= J_max/M;
V_rel   = 0.6; % Relative velocity at deployment (m/s)
t_drift = 4.0 * 90.0 * 60.0; % Drift time (s)
p = 0.7E-2;
d = 1;

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.001;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e*e), e, i, O, o, nu, const.mu);

n = utl_orbrate(a);      % Orbital rate
w_hill = [0.0; 0.0; n];


% Add initial velocity difference
r2 = r1;
v2 = randn(3, 1);
v2 = v1 + V_rel * (v2 / norm(v2));

% Allow spacecraft to drift apart
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift);




Q_transform = utl_eci2hill(r1, v1);
r_hill = Q_transform * (r2 - r1);
v_hill = Q_transform * (v2 - v1) - cross(w_hill, r_hill);
clear X;
X(:, 1) = [r_hill; v_hill];
N= 1000;
y_dvs=zeros(N,1);
deltaenergies=zeros(N,1);
for i = 1:(N - 1)
    i
    % Calculate our hill state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    %point B
    [~,potential,~]= env_gravity(0,r1);
    energy1= 0.5*dot(v1,v1)-potential;
    [~,potential,~]= env_gravity(0,r2);
    energy2= 0.5*dot(v2,v2)-potential;
    if (norm(r_hill(2))<10E3 && abs(energy1-energy2)<200)
        %do cw
        x_dv=-v_hill(1);
        y_x= r_hill(2);
        y_v= (-12*pi*r_hill(1)-1/n*6*pi*v_hill(2));
        y_maxu= 1/n*6*pi*sqrt(2)/2*max_dv_thrust;
        y_u= discrete_suicide_burn(y_maxu, y_v,y_x);
        y_dv= y_u/(-1/n*6*pi);
        dv=y_dv*v2/norm(v2);
    else
        %just use energy pd
        x_dv=0;
        %zero energy diff pd control
        pterm= p*r_hill(2)
        dterm= d*(energy1-energy2)
        dv=(pterm+dterm)*v2;
        dv= max(min(dv,sqrt(2)/2*max_dv_thrust),-sqrt(2)/2*max_dv_thrust);
    end
    u_now_hill= [x_dv;0;0;];
    % Apply actuation
    dv= Q_eci_hill' * u_now_hill + dv + normal_plane_correction(r1,v1,r2,v2,max_dv_thrust);
    dv= dv/norm(dv)*min(norm(dv),max_dv_thrust);
    v2 = v2 + dv;
    % Simulate dynamics
    dt_fire= 3*pi/2/n;
    [~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, dt_fire);
    % Calculate our hill state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    %point A
    [~,potential,~]= env_gravity(0,r1);
    energy1= 0.5*dot(v1,v1)-potential;
    [~,potential,~]= env_gravity(0,r2);
    energy2= 0.5*dot(v2,v2)-potential;
    if (norm(r_hill(2))<10E3 && abs(energy1-energy2)<200)
        %do cw
        x_dv= -n*(4*r_hill(1)+1/n*v_hill(1)+2/n*v_hill(2));
        x_dv=max(min(x_dv,sqrt(2)/2*max_dv_thrust),-sqrt(2)/2*max_dv_thrust);
        dv=0;
    else
        %just use energy
        x_dv=0;
        %zero energy diff
        %zero energy diff pd control
        pterm= p*r_hill(2)
        dterm= d*(energy1-energy2)
        dv=(pterm+dterm)*v2;
        dv= max(min(dv,sqrt(2)/2*max_dv_thrust),-sqrt(2)/2*max_dv_thrust);
    end
    u_now_hill= [x_dv;0;0];
    % Apply actuation
    dv= Q_eci_hill' * u_now_hill+dv+normal_plane_correction(r1,v1,r2,v2,max_dv_thrust);
    dv= dv/norm(dv)*min(norm(dv),max_dv_thrust);
    v2 = v2 + dv;
    % Simulate dynamics
    dt_fire= pi/2/n;
    [~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, dt_fire);
    % Calculate new state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    X(:, i + 1) = [r_hill; v_hill];
    [~,potential,~]= env_gravity(0,r1);
    energy1= 0.5*dot(v1,v1)-potential;
    [~,potential,~]= env_gravity(0,r2);
    energy2= 0.5*dot(v2,v2)-potential;
    deltaenergies(i)=energy2-energy1;
    
end

figure
hold on
plot(X(1, :), '-r')
plot(X(2, :), '-g')
plot(X(3, :), '-b')
hold off
title('Resulting Relative Position of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Position (m)')

figure
hold on
plot(X(4, :), '-r')
plot(X(5, :), '-g')
plot(X(6, :), '-b')
hold off
title('Resulting Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

figure;
plot(deltaenergies);

function normal_plane_dv2= normal_plane_correction(r1,v1,r2,v2,max_dv_thrust)
    %calc average angular momentum
    %[h1,h2] = average_angular_momentum(r1, v1, r2, v2);
    h1= cross(r1,v1);
    h2= cross(r2,v2);
    h2_hat= h2/norm(h2);
    dh2=dot(h2,h1)/dot(h1,h1)*h1-h2;
    normal_plane_dv= dot(cross(r2,h2_hat)/norm(cross(r2,h2_hat)),dh2)/norm(r2);
    normal_plane_dv= max(min(normal_plane_dv,sqrt(2)/2*max_dv_thrust),-sqrt(2)/2*max_dv_thrust);
    normal_plane_dv2= normal_plane_dv*h2_hat;
end


function u= discrete_suicide_burn(maxu, v,x)
%impements a discrete suicide burn 
%   calculates the required control u, to drive x and v to 0 asap.
%   the system is  x(k+1)= x(k)+v(k)+u(k)
%                   v(k+1)= v(k)+u(k)
%zero case
if(x==0 && v==0)
    "stoped";
    u=0;
    return;
end
%make x negative
signx= -sign(x);
x= x*signx;
v= v*signx;
if (v<=0)
    "moving away";
    u= -x-v;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%build min distance to stop.
kmin= floor(v/maxu);
residv= mod(v,maxu);
area= -x;
areamin= (kmin-1)*(kmin)/2*maxu+residv*kmin;
area= area-areamin;%residual area
maxdarea= kmin*(maxu-residv);
if (maxdarea>=area)
    "step 1";
    %implement step 1
    u=area/kmin-maxu;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%step 2, add step
kmin= kmin+1;
area= area-maxdarea;%residual area
maxdarea= kmin*maxu;
if (maxdarea>=area)
    "step 2";
    %implement step 2
    u=area/kmin-residv;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%step 3, add step
kmin= kmin+1;
area= area-maxdarea;%residual area
maxdarea= kmin*residv;
if (maxdarea>=area)
    "step 3";
    %partial speed ahead
    %implement step 3
    u=area/kmin+(maxu-residv);
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
"full speed ahead";
u= maxu;%full speed ahead
u= u*signx;
end
