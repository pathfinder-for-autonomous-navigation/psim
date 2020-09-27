function [out_state, J_ecef, phase_till_next_node] = make_mex_orbit_controller(t_last_firing, t, r_ecef, v_ecef, dr_ecef, dv_ecef)

    assert(isequal(size(t_last_firing),[1,1]), "t_last_firing must be a scalar")

    assert(isequal(size(t),[1,1]), "t must be a scalar")
    assert(isequal(size(r_ecef),[3,1]), "r_ecef must be a 3x1 vector")
    assert(isequal(size(v_ecef),[3,1]), "v_ecef must be a 3x1 vector")
    assert(isequal(size(dr_ecef),[3,1]), "dr_ecef must be a 3x1 vector")
    assert(isequal(size(dv_ecef),[3,1]), "dv_ecef must be a 3x1 vector")

    [out_state, J_ecef, phase_till_next_node] = control_orbit(t_last_firing, t, r_ecef, v_ecef, dr_ecef, dv_ecef);
end