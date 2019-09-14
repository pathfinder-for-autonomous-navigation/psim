function quat = utl_array2quaternion(in)
%UTL_ARRAY2QUATERNION converts a column vector length 4 to a quaternion.
%   the 4th component is real
quat= quaternion(in(4),in(1),in(2),in(3));
end

