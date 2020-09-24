function [orbit_controller] = make_mex_orbit_controller()

    orbit_controller.control = @control;

end

function [out_state, J_ecef, phase_till_next_node] = control(state, t, r_ecef, v_ecef, dr_ecef, dv_ecef)

    assert(isequal(size(t),[1,1]), "t must be a scalar")
    assert(isequal(size(r_ecef),[3,1]), "r_ecef must be a 3x1 vector")
    assert(isequal(size(v_ecef),[3,1]), "v_ecef must be a 3x1 vector")
    assert(isequal(size(dr_ecef),[3,1]), "dr_ecef must be a 3x1 vector")
    assert(isequal(size(dv_ecef),[3,1]), "dv_ecef must be a 3x1 vector")

    assert(isequal(size(state.t_last_firing),[1,1]), "state.t_last_firing must be a scalar")
    assert(isequal(size(state.this_r_ecef0),[3,1]), "state.this_r_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.that_r_ecef0),[3,1]), "state.that_r_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.this_r_hat),[3,1]), "state.this_r_hat must be a 3x1 vector")
    assert(isequal(size(state.this_v_ecef0),[3,1]), "state.this_v_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.that_v_ecef0),[3,1]), "state.that_v_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.this_v_hat),[3,1]), "state.this_v_hat must be a 3x1 vector")
    assert(isequal(size(state.this_h_ecef0),[3,1]), "state.this_h_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.that_h_ecef0),[3,1]), "state.that_h_ecef0 must be a 3x1 vector")
    assert(isequal(size(state.this_h_hat),[3,1]), "state.this_h_hat must be a 3x1 vector")
    assert(isequal(size(state.DCM_hill_ecef0),[3,3]), "state.DCM_hill_ecef0 must be a 3x3 vector")

    [out_state, J_ecef, phase_till_next_node] = control_orbit(state.t_last_firing, ... 
                                                                state.this_r_ecef0, state.that_r_ecef0, state.this_r_hat, ...
                                                                state.this_v_ecef0, state.that_v_ecef0, state.this_v_hat, ...
                                                                state.this_h_ecef0, state.that_h_ecef0, state.this_h_hat, ...
                                                                state.DCM_hill_ecef0, t, r_ecef, v_ecef, dr_ecef, dv_ecef);
end