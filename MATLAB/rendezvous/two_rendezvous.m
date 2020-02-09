clear all
global const
config();
M = 4.0; % Mass (kg)
J_min = 2e-4; % Min impulse (Ns)
J_max = 2e-2;%2e-2; % Max impulse (Ns)
max_dv_thrust= J_max/M;
V_rel   = 0.1; % Relative velocity at deployment (m/s)
t_drift = 24.0 * 90.0 * 60.0; % Drift time (s)

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.0;        % Eccentricity                          (unitless)
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
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift, 0);




Q_transform = utl_eci2hill(r1, v1);
r_hill = Q_transform * (r2 - r1);
v_hill = Q_transform * (v2 - v1) - cross(w_hill, r_hill);
clear X;
X(:, 1) = [r_hill; v_hill];
N= 1000;
y_dvs=zeros(N,1);
for i = 1:(N - 1)
    % Calculate our hill state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    %point B
    x_dv=-v_hill(1);
    x_dv= max(min(x_dv,max_dv_thrust),-max_dv_thrust);
    z_dv=-v_hill(3);
    z_dv= max(min(z_dv,max_dv_thrust),-max_dv_thrust);
    y_x= r_hill(2);
    y_v= (-12*pi*r_hill(1)-1/n*6*pi*v_hill(2));
    y_maxu= 1/n*6*pi*max_dv_thrust;
    y_u= discrete_suicide_burn(y_maxu, y_v,y_x);
    y_dv= y_u/(-1/n*6*pi);
    y_dv= max(min(y_dv,max_dv_thrust),-max_dv_thrust);
    y_dvs(i)=y_dv;
    u_now_hill= [x_dv;y_dv;z_dv;];
    dt_fire= 3*pi/2/n;
    % Apply actuation
    v2 = v2 + Q_eci_hill' * u_now_hill;
    % Simulate dynamics
    [~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, dt_fire, 0);
    % Calculate our hill state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    %point A
    x_dv= -n*(4*r_hill(1)+1/n*v_hill(1)+2/n*v_hill(2));
    x_dv= max(min(x_dv,max_dv_thrust),-max_dv_thrust);
    z_dv=-v_hill(3);
    z_dv= max(min(z_dv,max_dv_thrust),-max_dv_thrust);
    y_dv= 0;
    u_now_hill= [x_dv;y_dv;z_dv;];
    dt_fire= pi/2/n;
    % Apply actuation
    v2 = v2 + Q_eci_hill' * u_now_hill;
    % Simulate dynamics
    [~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, dt_fire, 0);
    % Calculate new state
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    X(:, i + 1) = [r_hill; v_hill];
    
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
plot(y_dvs/max_dv_thrust);


function [t, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift, plot_flag)
% Performs the drift phase of the simulation. Propegates the satellites
% orbits to 't_drift' seconds and plots the position and velocity of
% satellite two relative to satellite one if plot is true.

opt = odeset('RelTol', 1e-5, 'AbsTol', 1e-3);
[t, y] = ode113(@frhs, [0.0, t_drift], [r1; v1; r2; v2], opt);

if plot_flag
    
    dy = zeros(length(t), 6);
    for i = 1:length(t)
        Q = eci_to_lvlh(y(i, 1:3)', y(i, 4:6)');
        dy(i, 1:3) = (y(i, 7:9) - y(i, 1:3)) * Q';
        dy(i, 4:6) = (y(i, 10:12) - y(i, 4:6)) * Q';
    end
    
    figure
    hold on
    plot(t, dy(:, 1), '-r')
    plot(t, dy(:, 2), '-g')
    plot(t, dy(:, 3), '-b')
    hold off
    title('Drift Phase Relative Position of Satellite Two (rgb ~ xyz LVLH)')
    xlabel('Time (s)')
    ylabel('Position (m)')

end

t = t(end);
r1 = y(end, 1:3)';
v1 = y(end, 4:6)';
r2 = y(end, 7:9)';
v2 = y(end, 10:12)';

end


function dy = frhs(~, y)

dy = zeros(12, 1);

[gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = [gx; gy; gz];

[gx, gy, gz] = gravityzonal(y(7:9)', 'Earth', 4, 'Error');
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = [gx; gy; gz];

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
