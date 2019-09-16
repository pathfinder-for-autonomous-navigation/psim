function out = utl_quaternion2array(quat)
%UTL_QUATERNION2ARRAY converts a quaternion to a column vector length 4
%   the 4th component is real
assert(isa(quat,'quaternion'),'input must a quaternion')
q=compact(quat);
out= [q(2);q(3);q(4);q(1)];
end

