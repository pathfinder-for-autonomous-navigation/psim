clearvars; clc;

addpath('../');

global const

% initialize constants
config();
M = 3.7; % Mass (kg)
J_min = 2e-4; % Min impulse (Ns)
J_max = 5 * 2e-2; % Max impulse (Ns)
max_dv = J_max/M;
v_rel   = 1; % Relative velocity at deployment (m/s)
t_drift = 60.0 * 60.0; % Drift time (s)
p = 1.0e-2;
d = 5.0e-8;
h_gain = 5.0e-8;
thrust_noise_ratio = 0;
dt_fire_min = 10 * 60; % [s] minimum time between firings
% opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-2, 'InitialStep', 0.1);

% time
tmax = 200 * 60 * 60; % [s]
dt = 10; % [s]
t = 0 : dt : tmax; % [s]
N = length(t);

a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.001;      % Eccentricity                          (unitless)
I  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e*e), e, I, O, o, nu, const.mu);

n = utl_orbrate(a);      % [rad / s] Orbital rate
w_hill = [0.0; 0.0; n];


% Add initial velocity difference
r2 = r1;
% v2 = randn(3, 1);
v2 = [1; 0.5; 0.25];
v2 = v1 + v_rel * (v2 / norm(v2));

% Allow spacecraft to drift apart
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift);

% add measurement noise (zero for now)
r1_measure = r1; % +0.1*randn(3,1);
r2_measure = r2; % +0.1*randn(3,1);
v1_measure = v1; % +0.001*randn(3,1);
v2_measure = v2; % +0.001*randn(3,1);

% initialize arrays
X = zeros(6, N);
deltaenergies = zeros(N, 1);
u_vec = zeros(N, 1);
r_eci = zeros(6, N);
dv_vec = zeros(3, N);
r_fire = [];
dv_des = zeros(N, 1);
de_point_mass = zeros(N, 1);
energy1_vec = zeros(N, 1);
energy1_j1_vec = zeros(N, 1);
energy2_vec = zeros(N, 1);
energy2_j1_vec = zeros(N, 1);
dh_vec = zeros(3, N);
h1_vec = zeros(3, N);
h2_vec = zeros(3, N);

% Calculate initial state
Q_eci_hill = utl_eci2hill(r1, v1);
r_hill = Q_eci_hill * (r2 - r1);
v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
X(:, 1) = [r_hill; v_hill];
r_eci(:, 1) = [r1; r2];

% calculate energy
[~,potential,~]= env_gravity(0,r1);
energy1= 0.5*dot(v1,v1)-potential;
[~,potential,~]= env_gravity(0,r2);
energy2= 0.5*dot(v2,v2)-potential;
deltaenergies(1)=energy2-energy1;
% energy1_point_mass = norm(v1)^2 / 2 - const.mu / norm(r1);
% energy2_point_mass = norm(v2)^2 / 2 - const.mu / norm(r2);
% de_point_mass(1) = energy2_point_mass - energy1_point_mass;
energy1_vec(1) = energy1;
% energy1_j1_vec(1) = energy1_point_mass;
energy2_vec(1) = energy2;
% energy2_j1_vec(1) = energy2_point_mass;

% calculate orbital angular momentum
h1 = cross(r1, v1);
h2 = cross(r2, v2);
dh = h1 - h2;
dh_vec(:, 1) = dh;
h1_vec(:, 1) = h1;
h2_vec(:, 1) = h2;

t_fire = -dt_fire_min; % set so we fire on the first orbit

for i = 1 : N - 1
    
    if ~mod(i, 1000)
        fprintf('progress: step %d / %d\n', i, N - 1);
    end
    
    % calculate follower orbital elements
    a = -const.mu / (2 * energy2);
    n = utl_orbrate(a);
    M = n * t(i);
    M = mod(M, 2 * pi);
    
    % calculate eccentric anomaly
    del = 1;
    E = M / (1 - e);
    
    if E > sqrt(6 * (1 - e) / e)
        E = (6 * M / e) ^ (1/3);
    end
    
    while del > eps(2 * pi)
        
        E = E - (M - E + e * sin(E)) / (e * cos(E) - 1);
        del = max(abs(M - (E - e * sin(E))));
        
    end
    
    % check if follower is at a firing point
    if (abs(E - pi / 4) < 0.01 || abs(E - 3 * pi / 4) < 0.01) && t(i) - t_fire > dt_fire_min
        
        % record firing time and position
        t_fire = t(i);
        r_fire = [r_fire, [r1; r2]];
        
        % energy PD controller
        pterm = p * atan2(r_hill(2), r_hill(1)); %try -atan2(r_hill(2), r_hill(1)), or maybe actual phase angle of orbits
        dterm = -d * (energy2 - energy1);
        dv_energy = (pterm + dterm) * v2 / norm(v2);
%         dv = max(min(dv, sqrt(2) / 2 * max_dv), -sqrt(2) / 2 * max_dv);
%         u_now_hill = [0; 0; 0];
        
        % normal plane correction
        dhhat = dh / norm(dh); % instead calculate h2 - projection of h2 onto h1
        r2hat = r2 / norm(r2);
        Jhat_plane = cross(dhhat, r2hat);
        Jhat_plane = Jhat_plane / norm(Jhat_plane);
        J_plane = norm(dh) * Jhat_plane;
        dv_plane = h_gain * J_plane / M;
        
%         dv_plane = h_gain * normal_plane_correction(r1, v1, r2, v2, max_dv);
        
        dv = dv_energy + dv_plane;
        dv_des(i) = norm(dv);
        
        % Apply actuation
%         dv = Q_eci_hill' * u_now_hill + dv;
%         dv = dv / norm(dv) * min(norm(dv), max_dv);
        
        if norm(dv) > max_dv
            dv = max_dv * dv / norm(dv);
        end
        
%         if i == 5634
%             disp('5634')
%             pause;
%         end
        
        v2 = v2 + dv;
        dv_vec(:, i) = dv;
        
    end

    % simulate dynamics
    y = utl_ode4(@frhs, [0.0, dt], [r1; v1; r2; v2]);
%     [~, y] = ode45(@frhs, [0.0, dt], [r1; v1; r2; v2], opt);
    r1 = y(end, 1:3)';
    v1 = y(end, 4:6)';
    r2 = y(end, 7:9)';
    v2 = y(end, 10:12)';
    
    % Calculate new state
    w_hill = [0.0; 0.0; n];
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    X(:, i + 1) = [r_hill; v_hill];
    r_eci(:, i + 1) = [r1; r2];
    
    % calculate energies
    [~,potential,~]= env_gravity(0,r1);
    energy1= 0.5*dot(v1,v1)-potential;
    [~,potential,~]= env_gravity(0,r2);
    energy2= 0.5*dot(v2,v2)-potential;
    deltaenergies(i + 1)=energy2-energy1;
%     energy1_point_mass = norm(v1)^2 / 2 - const.mu / norm(r1);
%     energy2_point_mass = norm(v2)^2 / 2 - const.mu / norm(r2);
%     de_point_mass(i + 1) = energy2_point_mass - energy1_point_mass;
    energy1_vec(i + 1) = energy1;
    energy2_vec(i + 1) = energy2;
%     energy1_j1_vec(i + 1) = energy1_point_mass;
%     energy2_j1_vec(i + 1) = energy2_point_mass;

    % calculate orbital angular momentum
    h1 = cross(r1, v1);
    h2 = cross(r2, v2);
    dh = h1 - h2;
    dh_vec(:, i + 1) = dh;
    h1_vec(:, i + 1) = h1;
    h2_vec(:, i + 1) = h2;
    
end

% plots

% follower ECI position
figure;
plot(r_eci(4, :), r_eci(5, :)); hold on
plot(r_fire(4, :), r_fire(5, :), 'o')
xlabel('x ECI [m]')
ylabel('y ECI [m]')
title('follower ECI position')
legend('ECI position', 'firing points')

% relative position norm
figure
hold on
plot(t, X(1, :), '-r')
plot(t, X(2, :), '-g')
plot(t, X(3, :), '-b')
hold off
title('Relative Position of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Position (m)')

% relative position norm
rel_pos = zeros(1, N);

for i = 1 : N
    
    rel_pos(i) = norm(X(1 : 3, i));
    
end

figure;
plot(t, rel_pos)
xlabel('t [s]')
ylabel('position [m]')
title('relative position norm')

% relative velocity
figure
hold on
plot(t, X(4, :), '-r')
plot(t, X(5, :), '-g')
plot(t, X(6, :), '-b')
hold off
title('Relative Velocity of Satellite Two (rgb ~ xyz LVLH)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

% relative velocity norm
rel_vel = zeros(1, N);

for i = 1 : N
    
    rel_vel(i) = norm(X(4:6, i));
    
end

figure;
plot(t, rel_vel)
xlabel('t [s]')
ylabel('position [m/s]')
title('relative velocity norm')

% energy difference
figure;
plot(t, deltaenergies)
xlabel('t [s]')
title('delta energies')

% figure;
% plot(t, energy1_vec, 'b--'); hold on
% plot(t, energy1_j1_vec, 'b')
% plot(t, energy2_vec, 'r--')
% plot(t, energy2_j1_vec, 'r')
% xlabel('t [s]')
% ylabel('energy [J]')
% title('follower and leader energy')
% legend('follower J4', 'follower J1', 'follower J4', 'follower J1')

% dv
figure;
plot(dv_vec(1, :)); hold on
plot(dv_vec(2, :))
plot(dv_vec(3, :))
xlabel('t [s]')
ylabel('\Delta v [m/s]')
legend('x', 'y', 'z')
title('\Delta v')
grid on

dv_norm = zeros(N, 1);

for i = 1 : N
    
    dv_norm(i) = norm(dv_vec(:, i));
    
end

dv_total = cumsum(dv_norm);

figure;
plot(t, dv_norm)
xlabel('t [s]')
ylabel('\Delta v [m/s]')
title('\Delta v norm')

figure;
plot(t, dv_total)
xlabel('t [s]')
ylabel('\Delta v [m/s]')
title('Cumulative \Delta v')

figure;
plot(t, dv_des); hold on
plot(t, max_dv * ones(N, 1))
xlabel('t [s]')
ylabel('\Delta v [m/s]')
title('Desired \Delta v norm')
legend('commanded dv', 'max dv')

% angular momentum
figure;
plot(dh_vec(1, :)); hold on
plot(dh_vec(2, :))
plot(dh_vec(3, :))
xlabel('t [s]')
ylabel('h_1 - h_2')
title('angular momentum difference')
grid on

for i = 1 : N
    dh_norm(i) = norm(dh_vec(:, i));
    h1_norm(i) = norm(h1_vec(:, i));
    h2_norm(i) = norm(h2_vec(:, i));
end

figure;
plot(t, h1_norm); hold on
plot(t, h2_norm);
xlabel('t [s]')
ylabel('h')
title('angular momentums')
legend('h_1', 'h_2')

figure;
plot(t, dh_norm)
xlabel('t [s]')
ylabel('h')
title('angular momentum difference norm')

function dy = frhs(~, y)

% global const

dy = zeros(12, 1);

[gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = [gx; gy; gz];
% dy(4:6, 1) = -const.mu * y(1:3, 1) / norm(y(1:3, 1))^3;

[gx, gy, gz] = gravityzonal(y(7:9)', 'Earth', 4, 'Error');
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = [gx; gy; gz];
% dy(10:12, 1) = -const.mu * y(7:9, 1) / norm(y(7:9, 1))^3;

end

function normal_plane_dv2= normal_plane_correction(r1,v1,r2,v2,max_dv)

%calc average angular momentum
%[h1,h2] = average_angular_momentum(r1, v1, r2, v2);
h1 = cross(r1, v1);
h2 = cross(r2, v2);
h2_hat = h2 / norm(h2);
dh2 = dot(h2, h1) / dot(h1, h1) * h1 - h2;
normal_plane_dv = dot(cross(r2, h2_hat) / norm(cross(r2, h2_hat)), dh2) / norm(r2);
% normal_plane_dv = max(min(normal_plane_dv,sqrt(2)/2*max_dv_thrust),-sqrt(2)/2*max_dv_thrust);
normal_plane_dv2 = normal_plane_dv * h2_hat;

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
