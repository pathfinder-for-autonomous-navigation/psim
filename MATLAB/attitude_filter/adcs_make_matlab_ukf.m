function [ukf] = adcs_make_matlab_ukf()
ukf = struct()
ukf.reset = @reset;
ukf.triad_reset = @triad_reset;
ukf.update = @update;
end

function [state] = reset(q, gyro_bias, cov)
% TODO
end

function [state] = triad_reset(t, r_ecef, b_body, s_body)
% TODO
end

function [state, q_est, gyro_bias_est, cov_est] = update(state, t, r_ecef, b_body, s_body, w_body)
% TODO
end
