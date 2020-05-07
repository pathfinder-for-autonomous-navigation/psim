function [ukf] = adcs_make_mex_ukf()
ukf = struct()
ukf.reset = @reset;
ukf.triad_reset = @estimator_triad_reset;
ukf.update = @estimator_update;
end

function [state] = reset(t, q)
    state = estimator_reset(t, q);
end

function [state] = estimator_triad_reset(t, r_ecef, b_body, s_body)
% TODO
end

function [state, q_est, gyro_bias_est, cov_est] = estimator_update(state, t, r_ecef, b_body, s_body, w_body)
% TODO
end
