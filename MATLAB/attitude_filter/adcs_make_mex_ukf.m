function [ukf] = adcs_make_mex_ukf()
ukf.reset = @reset;
ukf.triad_reset = @triad_reset;
ukf.update = @update;
end

function [out_state] = reset(t, q)
    assert(isequal(size(t), [1,1]), "t must be a scalar")
    assert(isequal(size(q), [4,1]), "q must be a 4x1 quaternion")
    out_state = estimator_reset(t, q);
end

function [out_state] = triad_reset(t, r_ecef, b_body, s_body)
    assert(isequal(size(t), [1,1]), "t must be a scalar")
    assert(isequal(size(r_ecef),[3,1]), "r_ecef must be a 3x1 vector")
    assert(isequal(size(b_body),[3,1]), "b_body must be a 3x1 vector")
    assert(isequal(size(s_body),[3,1]), "s_body must be a 3x1 vector")

    [out_state] = estimator_triad_reset(t, r_ecef, b_body, s_body);
% TODO
end

function [out_state, q_est, gyro_bias_est, cov_est] = update(state, t, r_ecef, b_body, s_body, w_body)

    assert(isequal(size(t),[1,1]), "t must be a scalar")
    assert(isequal(size(r_ecef),[3,1]), "r_ecef must be a 3x1 vector")
    assert(isequal(size(b_body),[3,1]), "b_body must be a 3x1 vector")
    assert(isequal(size(s_body),[3,1]), "s_body must be a 3x1 vector")
    assert(isequal(size(w_body),[3,1]), "w_body must be a 3x1 vector")

    assert(isequal(size(state.q),[4,1]), "state.q must be a 4x1 quaternion")
    assert(isequal(size(state.x),[6,1]), "state.x must be a 6x1 vector")
    assert(isequal(size(state.P),[6,6]), "state.P must be a 6x6 vector")
    assert(isequal(size(state.t),[1,1]), "state.t must be a scalar")

    [out_state, q_est, gyro_bias_est, cov_est] = estimator_update(state.q, state.x, state.P, state.t, t, r_ecef, b_body, s_body, w_body);
end
