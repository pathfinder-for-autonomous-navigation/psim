function Q_eci2hill = utl_eci2hill(r, v)
% Determines the DCM from ECI to the hill frame around the provided
% position and velocity.

r_hat = -r / norm(r);
n_hat = cross(r, v) / norm(cross(r, v));
v_hat = cross(n_hat, r_hat);

Q_eci2hill = [r_hat'; v_hat'; n_hat'];

end
