function qout = utl_quat_conj(q)
%utl_quat_conj Congugate q. This reverses the direction of rotation.
% Quaternions have the forth component as scalar.

%#codegen 
qout= [-q(1:3);q(4)];
end
