function p = utl_quat2grp(q, a, f)
% Converts input quaternion to generalized
% rodriquez parameter representation
% Source: "Unscented Filtering for Spacecraft Attitude Estimation" 
% by Markley and Crasidis, 

p = q(1:3) * f / (a + q(4));