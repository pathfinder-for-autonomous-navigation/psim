function qout = utl_quat_cross_mult(q1,q2)
%utl_quat_cross_mult Cross multiply two quaternians result= q1xq2 see eq 2.82a.
%    This is the reverse of the normal quaterion multiplication.
%    Quaternions have the forth component as scalar.

%#codegen 
qout= zeros(4,1);
qout(4)=q1(4)*q2(4)-dot(q1(1:3),q2(1:3));
qout(1:3)= cross(q2(1:3),q1(1:3))+q1(4)*q2(1:3)+q2(4)*q1(1:3);
end
