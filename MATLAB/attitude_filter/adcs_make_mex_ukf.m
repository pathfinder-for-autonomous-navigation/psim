function [ukf] = adcs_make_mex_ukf()
ukf = struct()
ukf.reset = @reset;
ukf.triad_reset = @triad_reset;
ukf.update = @update;
end

function [state] = adcs_estimator_reset(t, q)
% TODO
end

function [state] = adcs_estimator_triad_reset(t, r_ecef, b_body, s_body)
% TODO
end

function [state, q_est, gyro_bias_est, cov_est] = adcs_estimator_update(state, t, r_ecef, b_body, s_body, w_body)
% TODO
end
