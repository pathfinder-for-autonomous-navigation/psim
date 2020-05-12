clearvars; clc;
addpath('../');
addpath('../utl');
addpath('../environmental_models');

generate_mex_code;

global const

% time
tmax = 5000; % [sec]
dt = 0.1; % [sec]
tspan = 0 : dt : tmax; % [sec]
N = length(tspan);

% physical constants
mu_earth = 3.986004415e14; % [m^3 / s^2]
r_earth = 6378137.0; % [m]
const.INITGPS_WN= 2045;% positive int, initial GPS week
const.INIT_DYEAR = decyear(utl_time2datetime(0.0, const.INITGPS_WN)); % decimal year for magnetic field model
const.e_earth = 0.0167086; % eccentricity of Earth's orbit
perihelion_date = datetime(2019,1,3,5,20,0,'TimeZone','UTCLeapSeconds');
const.tp_earth = utl_datetime2time(perihelion_date,const.INITGPS_WN);
[rp_earth,vp_earth] = planetEphemeris(juliandate(perihelion_date),'Sun','Earth');
rp_earth = rp_earth';
vp_earth = vp_earth';
h_earth = cross(rp_earth,vp_earth); % angular momentum of Earth
const.period_earth = 365.256363004*24*60*60; % [s]
const.quat_eci_perifocal = utl_triad(...
    [0; 0; 1], [1; 0; 0], h_earth / norm(h_earth), rp_earth / norm(rp_earth)...
    );
T0 = utl_time2datetime(0, const.INITGPS_WN); % pan epoch
T5 = T0 + years(5); % 5 years after pan epoch
dcm_ECEF0_ECI = dcmeci2ecef('IAU-2000/2006', [year(T0), month(T0), day(T0), hour(T0), minute(T0), second(T0)]);
dcm_ECEF5_ECI = dcmeci2ecef('IAU-2000/2006', [year(T5), month(T5), day(T5), hour(T5), minute(T5), second(T5)]);
polarprecessionaxis = -cross((dcm_ECEF0_ECI * dcm_ECEF5_ECI' * [0; 0; 1]), [0; 0; 1]);
const.PRECESSION_RATE = polarprecessionaxis / seconds(T5 - T0); % [rad / s] earth's rotation axis precession rate
const.quat_ecef0_eci = utl_quaternion2array(quaternion(dcm_ECEF0_ECI, 'rotmat', 'frame')); % ecef0 is ecef frame at time 0 inertialy stuck.
const.earth_rate_ecef = [0; 0; 7.2921158553E-5]; % [rad / s] earth's inertial rotation rate in ecef frame

% filter parameters
sv = 2.75e-4; % [rad / s^(1/2)] std of gyro noise (treated as process noise here)
su = 1e-6;
P = [(deg2rad(10))^2 * eye(3), zeros(3, 3);
   zeros(3, 3), (0.035 * 2)^2 * eye(3, 3)]; % initial covariance estimate

% sensor constants
gyro_bias_init = [-0.0344; 0.0279; 0.0144]; % [rad / s]
bias_est = [0 0 0]'; % [rad / s] initial estimate
R_mag = (5e-7)^2 * eye(3); % [T]
R_ss = (0.0349)^2 * eye(2); % [rad]
R = [R_ss, zeros(2, 3);
     zeros(3, 2), R_mag];

% initial conditions
sma = r_earth + 350e3; % [m]
e = 0; % circular orbit
I = deg2rad(35); % [rad]
omega = 0; % [rad]
Omega = 0; % [rad]
nu = 0; % [rad]
semi_p = sma * (1 - e^2); % [m]

[r0, v0] = utl_orb2rv(semi_p, e, I, Omega, omega, nu, mu_earth);


%initialize quaternion - True quaternion
th0 = deg2rad(30);
nhat0 = [1 0 0]';
q0 = [sin(th0/2) * nhat0; cos(th0/2)];
% th_est = deg2rad(30.5);
% nhat_est = [1 0 0]';
% q_est = [sin(th_est/2) * nhat_est; cos(th_est/2)];
% q_est = q0; % initialize with correct attitude estimate
%intialize true angular rate; should stay the same; play around with this;
%how high a rate can we initially estimate and still be good
IwB_B_0 = [0.01 0 0]'; % [rad / s]
% IwB_B_0 = [0 0 0]';

z0_orb = [r0; v0];
% z0_att = [q0; IwB_B_0];

% propagate true orbit dynamics - one call to ode45; 
% pre-compute orbital dynamics
% use these pre-computed states to call the filter
opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-3, 'InitialStep', 0.1);
[t, z] = ode45(@(t, z) ode_orb(t, z, mu_earth), tspan, z0_orb, opt);
r = z(:, 1:3)';
v = z(:, 4:6)';

% propagate true attitude dynamics
% switch this out in phase 2 for ode propogation
% add torques, precession, reaction wheels
gyro_noise = mvnrnd(zeros(1, 3), sv^2 * eye(3), N)';
IwB_B = repmat(IwB_B_0, 1, N) + gyro_noise;
q = zeros(4, N);
q(:, 1) = q0;

for i = 1 : N - 1
    
    psi = sin(0.5 * norm(IwB_B(:, i)) * dt) * IwB_B(:, i) / norm(IwB_B(:, i));
    psix = [0 -psi(3) psi(2);
        psi(3) 0 -psi(1);
        -psi(2) psi(1) 0];
    O = [cos(0.5 * norm(IwB_B(:, i)) * dt) * eye(3) - psix psi;
        -psi' cos(0.5 * norm(IwB_B(:, i)) * dt)];
    q(:, i + 1) = O * q(:, i);

end

% initial quaternion estimate
% passed into the filter
th_err_x_init = deg2rad(-5); %5 deg of error on EACH attitude axis
th_err_y_init = deg2rad(5);
th_err_z_init = deg2rad(5);
nhat_err_x_init = [1 0 0]';
nhat_err_y_init = [0 1 0]';
nhat_err_z_init = [0 0 1]';
%following are quaternion rotations
%testing: randomly generate error, expand error bounds in 120-122
q_err_x_init = [sin(th_err_x_init / 2) * nhat_err_x_init; cos(th_err_x_init / 2)];
q_err_y_init = [sin(th_err_y_init / 2) * nhat_err_y_init; cos(th_err_y_init / 2)];
q_err_z_init = [sin(th_err_z_init / 2) * nhat_err_z_init; cos(th_err_z_init / 2)];
q_est = utl_quat_cross_mult(q0, q_err_x_init);
q_est = utl_quat_cross_mult(q_est, q_err_y_init);
q_est = utl_quat_cross_mult(q_est, q_err_z_init);

% propagate gyro bias-truth
% add in temperature bias
bias_noise = mvnrnd(zeros(1, 3), su^2 * eye(3), N);
gyro_bias = zeros(3, N);
gyro_bias(:, 1) = gyro_bias_init;

for i = 1 : N - 1
    %euler step
    gyro_bias(:, i + 1) = gyro_bias(:, i) + dt * bias_noise(i + 1, :)';
    
end

% initialize arrays
w_meas_vec = zeros(3, N);
w_est_vec = zeros(3, N);
q_est_vec = zeros(4, N);
bias_est_vec = zeros(3, N);
P_vec = zeros(6, 6, N);
Knorm_vec = zeros(1, N);
inn_vec = zeros(5, N);
mag_meas_vec = zeros(3, N);
ss_meas_vec = zeros(2, N);
zbar_vec = zeros(5, N); % mag and 2-angle ss
xbar_vec = zeros(6, N);

q_est_vec(:, 1) = q_est;
bias_est_vec(:, 1) = bias_est;
P_vec(:, :, 1) = P;

%%%all the following are initializing truth for propogation later
% gyro measurement model; TRUTH: true measurement from the gyro
w_meas = IwB_B(:, 1) + gyro_bias(:, 1); % + mvnrnd(zeros(1, 3), sv^2 * eye(3), 1)';
w_meas_vec(:, 1) = w_meas;
w_est_vec(:, 1) = w_meas - bias_est;

% sun sensor measurement model
sat2sun_eci = env_sun_vector(tspan(1));
sat2sun_body = utl_rotateframe(q0, sat2sun_eci')';
th_ss_body = atan(sat2sun_body(2) / sat2sun_body(1)); % [rad]
phi_ss_body = acos(sat2sun_body(3)); % [rad]
ss_noise = mvnrnd(zeros(1, 2), R_ss, 1);
ss_ang_body = [th_ss_body; phi_ss_body] + ss_noise'; % add noise
ss_meas_vec(:, 1) = ss_ang_body;

% magnetometer measurement model
[quat_ecef_eci, rate_ecef] = env_earth_attitude( tspan(1) );
quat_eci_ecef = utl_quat_conj(quat_ecef_eci);
quat_body_ecef = utl_quat_cross_mult(q0, quat_eci_ecef);
r_ecef = utl_rotateframe( quat_ecef_eci, r0 )';
B_ecef = env_magnetic_field(tspan(1), r_ecef);
B_body = utl_rotateframe(quat_body_ecef, B_ecef')';
mag_noise = mvnrnd(zeros(1, 3), R_mag, 1);
B_body_meas = B_body + mag_noise'; % add noise
mag_meas_vec(:, 1) = B_body_meas;

% state contains info that persists across calls to the filter.
% attitude estimate quaternion, gyro bias estimate, and state covariance estimate
% uncomment filter implementation you want to look at

ukf = adcs_make_mex_ukf(); %C++ implementation
%ukf = adcs_make_matlab_ukf(); %MATLAB implementation

% reset takes time since epoch... so i replaced T0 with 0
state = ukf.reset(t(1), q_est);

for i = 1 : N - 1
    
    if ~mod(tspan(i), 100)
        fprintf('Progress: %.2f / %.2f [s] \n', tspan(i), tspan(end))
    end

    % gyro measurement model
    w_meas = IwB_B(:, i + 1) + gyro_bias(:, i + 1); % + mvnrnd(zeros(1, 3), sv^2 * eye(3), 1)';
    w_meas_vec(:, i + 1) = w_meas;
    
    % sun sensor measurement model
    sat2sun_body = utl_rotateframe(q(:, i + 1), env_sun_vector(t(i))')';
    th_ss_body = atan(sat2sun_body(2) / sat2sun_body(1)); % [rad]
    phi_ss_body = acos(sat2sun_body(3)); % [rad]
    ss_noise = mvnrnd(zeros(1, 2), R_ss, 1);
    ss_ang_body = [th_ss_body; phi_ss_body] + ss_noise'; % add noise

    % shihao added this: assuming theta in plane, phi comes down from top, idk if right
    ss_vec_body = [cos(ss_ang_body(1))*sin(ss_ang_body(2)); sin(ss_ang_body(1))*sin(ss_ang_body(2)); cos(ss_ang_body(2))];
%     ss_vec_body = sat2sun_body;

    % magnetometer measurement model
    [quat_ecef_eci, rate_ecef] = env_earth_attitude( tspan(i + 1) );
    quat_eci_ecef = utl_quat_conj(quat_ecef_eci);
    r_ecef = utl_rotateframe( quat_ecef_eci, r(:, i + 1) )';
    B_ecef = env_magnetic_field(tspan(i + 1), r_ecef);
    quat_body_ecef = utl_quat_cross_mult(q(:, i + 1), quat_eci_ecef);
    B_body = utl_rotateframe(quat_body_ecef, B_ecef')';
    mag_noise = mvnrnd(zeros(1, 3), R_mag, 1);
    B_body_meas = B_body + mag_noise';
   
    [state, q_est, gyro_bias_est, cov_est] = ukf.update(state, t(i), r_ecef, B_body_meas, ss_vec_body, w_meas);
    
    % vectors for plotting
    q_est_vec(:, i + 1) = q_est; %from filter
    bias_est_vec(:, i + 1) = gyro_bias_est; %from filter
    w_est_vec(:, i + 1) = w_meas - gyro_bias_est;%from filter
    P_vec(:, :, i + 1) = cov_est; %from filter cov_est from filter
    
end

% plots

% ECI position
figure;
plot(r(1, :), r(2, :))
xlabel('x [km]')
ylabel('y [km]')
title('ECI position')

% true quaternion components
figure;
plot(tspan, q(1, :)); hold on
plot(tspan, q(2, :))
plot(tspan, q(3, :))
plot(tspan, q(4, :))
xlabel('t [s]')
title('true quaternion components')

% estimated quaternion components
figure;
plot(tspan, q_est_vec(1, :)); hold on
plot(tspan, q_est_vec(2, :))
plot(tspan, q_est_vec(3, :))
plot(tspan, q_est_vec(4, :))
xlabel('t [s]')
title('estimated quaternion components')

% angular velocity
figure;
plot(tspan, w_meas_vec(1, :), 'r'); hold on
plot(tspan, w_meas_vec(2, :), 'r')
plot(tspan, w_meas_vec(3, :), 'r')
% plot(tspan, IwB_B(1, :), 'b')
% plot(tspan, IwB_B(2, :), 'b')
% plot(tspan, IwB_B(3, :), 'b')
plot(tspan, w_est_vec(1, :), 'k')
plot(tspan, w_est_vec(2, :), 'k')
plot(tspan, w_est_vec(3, :), 'k')
xlabel('t [s]')
ylabel('\omega [rad/s]')
title('angular velocity')
legend('\omega_{x, meas}', '\omega_{y, meas}', '\omega_{z, meas}',...
    '\omega_x', '\omega_y', '\omega_z',...
    '\omega_{x, est}', '\omega_{y, est}', '\omega_{z, est}')

% gyro bias
figure;
plot(tspan / 3600, bias_est_vec(1, :), 'r'); hold on
plot(tspan / 3600, bias_est_vec(2, :), 'b')
plot(tspan / 3600, bias_est_vec(3, :), 'k')
plot(tspan / 3600, gyro_bias(1, :), 'r--')
plot(tspan / 3600, gyro_bias(2, :), 'b--')
plot(tspan / 3600, gyro_bias(3, :), 'k--')
xlabel('t [hr]')
ylabel('\beta [rad/s]')
title('gyro bias')
legend('\beta_x, est', '\beta_y, est', '\beta_z, est',...
    '\beta_x', '\beta_y', '\beta_z')

% covariance norm
Pnorm_vec = zeros(1, size(P_vec, 3));

for i = 1 : length(Pnorm_vec)
    
    Pnorm_vec(i) = norm(P_vec(:, :, i));
    
end

figure;
plot(tspan, Pnorm_vec)
xlabel('t [s]')
ylabel('norm(P)')
title('covariance norm')

% attitude error angle
q_err = zeros(4, size(q, 2));
th_err = zeros(size(q, 2), 1);

for i = 1 : size(q_err, 2)
    
    q_err(:, i) = utl_quat_cross_mult( q_est_vec(:, i), utl_quat_conj(q(:, i)) );
    th_err(i) = 2 * asin( norm( q_err(1:3, i) ) );
    
end

figure;
semilogy(tspan, rad2deg(th_err)); hold on
xlabel('t [s]')
ylabel('error angle [deg]')
title('attitude knowledge error')
grid on

% error bounds
p_err = zeros(3, length(tspan));

a = 1; 
f = 4; 

for i = 1 : length(tspan)
    p_err(:, i) = utl_quat2grp(q_err(:, i), a, f);
end

p1_bound = 3 * squeeze(sqrt(P_vec(1, 1, :)));
p2_bound = 3 * squeeze(sqrt(P_vec(2, 2, :)));
p3_bound = 3 * squeeze(sqrt(P_vec(3, 3, :)));

figure;
plot(tspan, p_err(1, :), 'b'); hold on
plot(tspan, p1_bound, 'r')
plot(tspan, -p1_bound, 'r')
xlabel('t [s]')
ylabel('p_1 error')
title('GRP component 1 attitude error')

figure;
plot(tspan, p_err(2, :), 'b'); hold on
plot(tspan, p2_bound, 'r')
plot(tspan, -p2_bound, 'r')
xlabel('t [s]')
ylabel('p_2 error')
title('GRP component 2 attitude error')

figure;
plot(tspan, p_err(3, :), 'b'); hold on
plot(tspan, p3_bound, 'r')
plot(tspan, -p3_bound, 'r')
xlabel('t [s]')
ylabel('p_3 error')
title('GRP component 3 attitude error')

err_bias = gyro_bias - bias_est_vec;
x_bound = 3 * squeeze(sqrt(P_vec(4, 4, :)));
y_bound = 3 * squeeze(sqrt(P_vec(5, 5, :)));
z_bound = 3 * squeeze(sqrt(P_vec(6, 6, :)));

figure;
plot(tspan, err_bias(1, :), 'b'); hold on
plot(tspan, x_bound, 'r')
plot(tspan, -x_bound, 'r')
xlabel('t [s]')
ylabel('gyro bias x error [rad / s]')
title('gyro bias x 3\sigma error bounds')

figure;
plot(tspan, err_bias(2, :), 'b'); hold on
plot(tspan, y_bound, 'r')
plot(tspan, -y_bound, 'r')
xlabel('t [s]')
ylabel('gyro bias x error [rad / s]')
title('gyro bias y 3\sigma error bounds')

figure;
plot(tspan, err_bias(3, :), 'b'); hold on
plot(tspan, z_bound, 'r')
plot(tspan, -z_bound, 'r')
xlabel('t [s]')
ylabel('gyro bias x error [rad / s]')
title('gyro bias z 3\sigma error bounds')

% rodrigues parameter estimation
p_vec = zeros(3, N);
p_est_vec = zeros(3, N);

for i = 1 : N
    
    p_vec(:, i) = utl_quat2grp(q(:, i), a, f);
    p_est_vec(:, i) = utl_quat2grp(q_est_vec(:, i), a, f);
    
end

figure;
plot(tspan, p_est_vec(1, :), 'r'); hold on
plot(tspan, p_est_vec(2, :), 'b')
plot(tspan, p_est_vec(3, :), 'k')
plot(tspan, p_vec(1, :), 'r--')
plot(tspan, p_vec(2, :), 'b--')
plot(tspan, p_vec(3, :), 'k--')
xlabel('t [s]')
ylabel('GRP')
title('GRP')
legend('p_x, est', 'p_y, est', 'p_z, est',...
    'p_x', 'p_y', 'p_z')

% integration functions
function dzdt = ode_orb(t, z, mu_earth)

dzdt = zeros(6, 1);

dzdt(1:3, 1) = z(4:6, 1);
dzdt(4:6, 1) = (-mu_earth / norm(z(1:3, 1))^3) * z(1:3, 1);

end

function dzdt = ode_att(t, z)

dzdt = zeros(7, 1);

dzdt(1:4, 1) = utl_quat_deriv(z(1:4, 1), z(5:7, 1));
dzdt(5:7, 1) = zeros(3, 1); % torque-free motion

end