function [ukf] = adcs_make_matlab_ukf()
    ukf.reset = @reset;
    ukf.triad_reset = @triad_reset;
    ukf.update = @update;
end

function state = reset(q_est,bias_est,P)

    state.q = q_est; %estimate quaternion
    state.b = bias_est; %gyro bias estimate
    state.P = P; %state covariance estimate

end

function [state] = update(state, B_body_meas, w_meas,...
    dt,tspan,r_ecef, quat_eci_ecef,R)
    
    sv = 2.75e-4; % [rad / s^(1/2)] std of gyro noise (treated as process noise here)
    su = 1e-6;

    %sensor constants
    R_mag = (5e-7)^2 * eye(3); % [T]
    R_ss = (0.0349)^2 * eye(2); % [rad]
    R = [R_ss, zeros(2, 3);
     zeros(3, 2), R_mag];
 
    Qbar = 1E3*(dt / 2) * [(sv^2 - (su^2 * dt^2) / 6) * eye(3), zeros(3, 3);
        zeros(3, 3), su^2 * eye(3)]; % additive process noise matrix
    a = 1; % scaling for generalized rodrigues parameters
    f = 2 * (a + 1); % more scaling
    lam = 1;
 
    % initial filter state
    xhat = [0; 0; 0; state.b]; % initialize attitude error to zero
    
    % generate sigma points
    S = chol(state.P + Qbar);
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
        
        q_pert(ii, :) = utl_quat_cross_mult(sigmas_q(ii, :)', state.q);
        
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
    sat2sun_eci = env_sun_vector( tspan );
    B_ecef = env_magnetic_field(tspan, r_ecef);
    
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

    state.q = q_est; %estimate quaternion
    state.b = bias_est; %gyro bias estimate
    state.P = P; %state covariance estimate

end

function [state] = triad_reset(t, r_ecef, b_body, s_body)
% TODO
end
