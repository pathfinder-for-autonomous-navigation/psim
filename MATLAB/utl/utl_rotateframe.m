function u = utl_rotateframe(q,v)
%UTL_ROTATEFRAME quaternion frame rotation
%  U = ROTATEFRAME(Q,V) rotates the frame of reference for the three vector V
%  using quaternion Q stored as an array with 4th component real. 

%#codegen 


u= v+cross(2*q(1:3),cross(q(1:3),v)-q(4)*v);
end
