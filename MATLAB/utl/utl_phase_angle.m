function phase = utl_phase_angle(r1,v1,r2)
% utl_phase_angle the phase of r2 in the r1,v1 orbit plane from r1
% the phase is the relative of 2 with respect to 1.
% the vectors should be in any inertial frame with any units, 
% the orbit plane must be well defined, ie, norm(cross(r1,v1))>0
% phase is in radians range [-pi,pi]
DCM_hill_intertial = utl_inertial2hill(r1, v1);
r2_hill= DCM_hill_intertial*r2;
phase= atan2(r2_hill(2),r2_hill(1));
end

