function q = utl_grp2quat(p, a, f)
% Converts input p which is a 3-component generalized rodriques parameter
% attitude representation, to a quaternion
% Source: "Unscented Filtering for Spacecraft Attitude Estimation" 
% by Markley and Crasidis, 

q = zeros(4, 1);

q(4, 1) = (-a * norm(p)^2 + f * sqrt(f^2 + (1 - a^2) * norm(p)^2)) / (f^2 + norm(p)^2);
q(1:3, 1) = ((a + q(4)) / f) * p;