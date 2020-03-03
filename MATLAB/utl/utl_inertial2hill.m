function DCM_hill_inertial = utl_inertial2hill(r, v)
% Determines the DCM from ECI to the hill frame around the provided
% position and velocity (the position and velocity must be from an inertial
% frame).

r_hat = r / norm(r);
n_hat = cross(r, v) / norm(cross(r, v));
v_hat = cross(n_hat, r_hat);

DCM_hill_inertial = [r_hat'; v_hat'; n_hat'];

end
