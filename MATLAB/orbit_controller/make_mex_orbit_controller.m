function [J_ecef, phase_till_next_node] = make_mex_orbit_controller(t, r_ecef, v_ecef, dr_ecef, dv_ecef)
    
    assert(isequal(size(t),[1,1]), "t must be a scalar")
    assert(isequal(size(r_ecef),[3,1]), "r_ecef must be a 3x1 vector")
    assert(isequal(size(v_ecef),[3,1]), "v_ecef must be a 3x1 vector")
    assert(isequal(size(dr_ecef),[3,1]), "dr_ecef must be a 3x1 vector")
    assert(isequal(size(dv_ecef),[3,1]), "dv_ecef must be a 3x1 vector")

    J_ecef = control_orbit(t, r_ecef, v_ecef, dr_ecef, dv_ecef);
end