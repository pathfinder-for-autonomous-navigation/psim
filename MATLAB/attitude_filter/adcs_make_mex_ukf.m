function [ukf] = adcs_make_mex_ukf()
ukf = struct()
ukf.reset = @reset;
ukf.triad_reset = @triad_reset;
ukf.update = @update;
end

function [state] = reset(state, q, gyro_bias, cov)
% TODO
end

function [state] = triad_reset(state, t, r_ecef, b_body, s_body)
% TODO
end

function [state] = update(state, t, r_ecef, b_body, s_body, w_body)
% TODO
end
