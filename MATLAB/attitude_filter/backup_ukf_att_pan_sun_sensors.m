clearvars; clc;
addpath('../');
addpath('../utl');
addpath('../environmental_models');

generate_mex_code;

global const

% time
tmax = 1500; % [sec]
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
% su = deg2rad(0.0015); % [rad / s^(3/2)] std of gyro bias (treated as process noise here)
P = [(deg2rad(10))^2 * eye(3), zeros(3, 3);
    zeros(3, 3), (0.035 * 2)^2 * eye(3, 3)]; % initial covariance estimate
Qbar = 1e3 * (dt / 2) * [(sv^2 - (su^2 * dt^2) / 6) * eye(3), zeros(3, 3);
    zeros(3, 3), su^2 * eye(3)]; % additive process noise matrix
a = 1; % scaling for generalized rodrigues parameters
f = 2 * (a + 1); % more scaling
lam = 1;

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

th0 = deg2rad(30);
nhat0 = [1 0 0]';
q0 = [sin(th0/2) * nhat0; cos(th0/2)];
% th_est = deg2rad(30.5);
% nhat_est = [1 0 0]';
% q_est = [sin(th_est/2) * nhat_est; cos(th_est/2)];
% q_est = q0; % initialize with correct attitude estimate

IwB_B_0 = [0.01 0 0]'; % [rad / s]
% IwB_B_0 = [0 0 0]';

z0_orb = [r0; v0];
% z0_att = [q0; IwB_B_0];

% propagate true orbit dynamics
opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-3, 'InitialStep', 0.1);
[t, z] = ode45(@(t, z) ode_orb(t, z, mu_earth), tspan, z0_orb, opt);
r = z(:, 1:3)';
v = z(:, 4:6)';

% propagate true attitude dynamics
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
th_err_x_init = deg2rad(-5);
th_err_y_init = deg2rad(5);
th_err_z_init = deg2rad(5);
nhat_err_x_init = [1 0 0]';
nhat_err_y_init = [0 1 0]';
nhat_err_z_init = [0 0 1]';
q_err_x_init = [sin(th_err_x_init / 2) * nhat_err_x_init; cos(th_err_x_init / 2)];
q_err_y_init = [sin(th_err_y_init / 2) * nhat_err_y_init; cos(th_err_y_init / 2)];
q_err_z_init = [sin(th_err_z_init / 2) * nhat_err_z_init; cos(th_err_z_init / 2)];
q_est = utl_quat_cross_mult(q0, q_err_x_init);
q_est = utl_quat_cross_mult(q_est, q_err_y_init);
q_est = utl_quat_cross_mult(q_est, q_err_z_init);

% propagate gyro bias
bias_noise = mvnrnd(zeros(1, 3), su^2 * eye(3), N);
gyro_bias = zeros(3, N);
gyro_bias(:, 1) = gyro_bias_init;

for i = 1 : N - 1

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

% gyro measurement model
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

for i = 1 : N - 1
    
    if ~mod(tspan(i), 100)
        fprintf('Progress: %.2f / %.2f [s] \n', tspan(i), tspan(end))
    end
    
    % initial filter state
    xhat = [0; 0; 0; bias_est]; % initialize attitude error to zero
    
    % generate sigma points
    S = chol(P + Qbar);
    n = size(S, 2);
    sigmas = zeros(2 * n + 1, 6);
    sigmas(1, :) = xhat';
    
    for ii = 2 : n + 1
        sigmas(ii, :) = xhat' + sqrt(n + lam) * S(:, ii - 1)';
    end
    for ii = n + 2 : 2 * n + 1
        sigmas(ii, :) = xhat' - sqrt(n + lam) * S(:, ii - n - 1)';
    end

    % convert sigma points from GRPs to error quaternions
    sigmas_q = zeros(size(sigmas, 1), 4);
    
    for ii = 1 : size(sigmas, 1)
        
        psig = sigmas(ii, 1:3);
        qsig = utl_grp2quat(psig, a, f);
        sigmas_q(ii, :) = qsig';
        
    end
    
    % perturb current quaternion estimate
    q_pert = zeros(size(sigmas_q, 1), 4);
    
    for ii = 1 : size(sigmas_q, 1)
        
        q_pert(ii, :) = utl_quat_cross_mult(sigmas_q(ii, :)', q_est);
        
    end
    
    % propagate sigma points
    q_new = zeros(size(sigmas, 1), 4);
    
    for ii = 1 : size(sigmas, 1)
        
        w_est = w_meas - sigmas(ii, 4:6)';
        psi = sin(0.5 * norm(w_est) * dt) * w_est / norm(w_est);
        psix = [0 -psi(3) psi(2);
            psi(3) 0 -psi(1);
            -psi(2) psi(1) 0];
        O = [cos(0.5 * norm(w_est) * dt) * eye(3) - psix psi;
            -psi' cos(0.5 * norm(w_est) * dt)];
        q_new(ii, :) = O * q_pert(ii, :)';
        
    end
    
    % expected measurements for each sigma point
    sat2sun_eci = env_sun_vector( tspan(i + 1) );
    [quat_ecef_eci, rate_ecef] = env_earth_attitude( tspan(i + 1) );
    r_ecef = utl_rotateframe( quat_ecef_eci, r(:, i + 1) )';
    B_ecef = env_magnetic_field(tspan(i + 1), r_ecef);
    
    exp_meas = zeros(5, size(sigmas_q, 1)); % mag and 2-angle ss model
    
    for ii = 1 : size(sigmas_q, 1)
        
        sat2sun_body_exp = utl_rotateframe(q_new(ii, :)', sat2sun_eci')';
        th_ss_body = atan(sat2sun_body_exp(2) / sat2sun_body_exp(1)); % [rad]
        phi_ss_body = acos(sat2sun_body_exp(3)); % [rad]
        ss_ang_body = [th_ss_body; phi_ss_body];

        quat_body_ecef = utl_quat_cross_mult(q_new(ii, :)', quat_eci_ecef);
        B_body_exp = utl_rotateframe(quat_body_ecef, B_ecef')';
        exp_meas(:, ii) = [ss_ang_body; B_body_exp]; % mag and 2-angle ss
        
    end
    
    % calculate propagated error quaternions
    dq_new = zeros(size(sigmas_q, 1), 4);
    
    for ii = 1 : size(sigmas_q, 1)
        
        dq_new(ii, :) = utl_quat_cross_mult( q_new(ii, :)', utl_quat_conj(q_new(1, :)') );
        
    end
    
    % convert back to GRPs to get the propagated sigma points
    sigmas_new = zeros(size(sigmas, 1), 6);
    sigmas_new(1, 4:6) = sigmas(1, 4:6);
    
    for ii = 2 : size(sigmas, 1)
        
        p_new = utl_quat2grp(dq_new(ii, :), a, f);
        sigmas_new(ii, 1:3) = p_new';
        sigmas_new(ii, 4:6) = sigmas(ii, 4:6); % gyro bias
        
    end

    % calculate weights for each sigma point
    weights = zeros(size(sigmas, 1), 1);
    weights(1) = lam / (n + lam);
    
    for ii = 2 : length(weights)
        
        weights(ii) = 1 / (2 * (n + lam));
        
    end
    
    % compute weighted mean and mean expected measurement
    xbar = zeros(6, 1);
    zbar = zeros(5, 1); % mag and 2-angle ss

    for ii = 1 : length(weights)
        
        xbar = xbar + weights(ii) * sigmas_new(ii, :)';
        zbar = zbar + weights(ii) * exp_meas(:, ii);
        
    end
    
    % calculate weights for covariance estimate
    weights_cov = zeros(size(sigmas, 1), 1);
    weights_cov(1) = lam / (n + lam); % + (1 - alpha^2 + beta);
    
    for ii = 2 : length(weights_cov)
        
        weights_cov(ii) = 1 / (2 * (n + lam));
        
    end
    
    % calculate covariances
    P_pred = zeros(6, 6); % predicted covariance
    Pyy = zeros(5, 5); % output covariance
    Pxy = zeros(6, 5); % cross-correlation covariance

    for ii = 1 : length(weights_cov)
        
        sigma_diff = sigmas_new(ii, :)' - xbar;
        meas_diff = exp_meas(:, ii) - zbar;
        
        P_pred = P_pred + weights_cov(ii) * (sigma_diff * sigma_diff');
        Pyy = Pyy + weights_cov(ii) * (meas_diff * meas_diff');
        Pxy = Pxy + weights_cov(ii) * (sigma_diff * meas_diff');
        
    end
    
    P_pred = P_pred + Qbar; % add process noise to predicted covariance
    Pvv = Pyy + R; % innovation covariance
    
    % compute Kalman gain
    K = Pxy * inv(Pvv);
    
    % gyro measurement model
    w_meas = IwB_B(:, i + 1) + gyro_bias(:, i + 1); % + mvnrnd(zeros(1, 3), sv^2 * eye(3), 1)';
    w_meas_vec(:, i + 1) = w_meas;
    
    % sun sensor measurement model
    sat2sun_body = utl_rotateframe(q(:, i + 1), sat2sun_eci')';
    th_ss_body = atan(sat2sun_body(2) / sat2sun_body(1)); % [rad]
    phi_ss_body = acos(sat2sun_body(3)); % [rad]
    ss_noise = mvnrnd(zeros(1, 2), R_ss, 1);
    ss_ang_body = [th_ss_body; phi_ss_body] + ss_noise'; % add noise

    % magnetometer measurement model
    quat_body_ecef = utl_quat_cross_mult(q(:, i + 1), quat_eci_ecef);
    B_body = utl_rotateframe(quat_body_ecef, B_ecef')';
    mag_noise = mvnrnd(zeros(1, 3), R_mag, 1);
    B_body_meas = B_body + mag_noise';
    
    % update estimate based on measurements
    meas = [ss_ang_body; B_body_meas];
    xhat = xbar + K * (meas - zbar);
    P = P_pred - (K * Pvv * K');
    
    % convert GRP estimate to quaternion
    dq_est = utl_grp2quat(xhat(1:3, 1), a, f);
    
    % update quaternion estimate
    q_est = utl_quat_cross_mult(dq_est, q_new(1, :)');
    
    % update gyro bias estimate
    bias_est = xhat(4:6, 1);
    
    % vectors for plotting
    q_est_vec(:, i + 1) = q_est;
    bias_est_vec(:, i + 1) = bias_est;
    w_est_vec(:, i + 1) = w_meas - bias_est;
    Knorm_vec(i + 1) = norm(norm(K));
    P_vec(:, :, i + 1) = P;
    inn_vec(:, i + 1) = meas - zbar;
    xbar_vec(:, i + 1) = xbar;
    zbar_vec(:, i + 1) = zbar;
    mag_meas_vec(:, i + 1) = B_body_meas;
    ss_meas_vec(:, i + 1) = ss_ang_body;
    
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
plot(tspan, IwB_B(1, :), 'b')
plot(tspan, IwB_B(2, :), 'b')
plot(tspan, IwB_B(3, :), 'b')
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

% kalman gain norm
figure;
plot(tspan, Knorm_vec)
xlabel('t [s]')
ylabel('norm(K)')
title('kalman gain norm')

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

% magnetometer measurements
figure;
plot(tspan, mag_meas_vec(1, :)); hold on
plot(tspan, mag_meas_vec(2, :))
plot(tspan, mag_meas_vec(3, :))
xlabel('t [s]')
ylabel('B [T]')
title('magnetometer measurements')
legend('b_1', 'b_2', 'b_3')

% mean expected mag measurement
figure;
plot(tspan, zbar_vec(3, :)); hold on
plot(tspan, zbar_vec(4, :))
plot(tspan, zbar_vec(5, :))
xlabel('t [s]')
ylabel('B [T]')
title('mean expected mag measurement')
legend('b_1', 'b_2', 'b_3')

% mag innovation
figure;
plot(tspan, inn_vec(3, :)); hold on
plot(tspan, inn_vec(4, :))
plot(tspan, inn_vec(5, :))
xlabel('t [s]')
ylabel('B [T]')
title('mag innovation')
legend('b_1', 'b_2', 'b_3')

% ss measurements
figure;
plot(tspan, ss_meas_vec(1, :)); hold on
plot(tspan, ss_meas_vec(2, :))
xlabel('t [s]')
ylabel('angle [rad]')
title('sun sensor measurements')
legend('\theta', '\phi')

% mean expected ss measurement
figure;
plot(tspan, zbar_vec(1, :)); hold on
plot(tspan, zbar_vec(2, :))
xlabel('t [s]')
ylabel('angle [rad]')
title('mean expected sun sensor measurement')
legend('\theta', '\phi')

% ss innovation
figure;
plot(tspan, inn_vec(1, :)); hold on
plot(tspan, inn_vec(2, :))
xlabel('t [s]')
ylabel('angle [rad]')
title('sun sensor innovation')
legend('\theta', '\phi')

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
