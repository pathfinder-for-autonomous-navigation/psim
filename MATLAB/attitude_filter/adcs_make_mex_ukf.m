function [ukf] = adcs_make_mex_ukf()
ukf.reset = @reset;
ukf.triad_reset = @triad_reset;
ukf.update = @update;
end

function [state] = reset(t, q)
    state = estimator_reset(t, q);
end

function [state] = triad_reset(t, r_ecef, b_body, s_body)
% TODO
end

function [out_state, q_est, gyro_bias_est, cov_est] = update(state, t, r_ecef, b_body, s_body, w_body)
    % feed each part of state manually in
    [out_state, q_est, gyro_bias_est, cov_est] = estimator_update(state.q, state.x, state.P, state.t, t, r_ecef, b_body, s_body, w_body);
end
