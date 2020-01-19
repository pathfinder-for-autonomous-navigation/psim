q1=[1;2;3;4;];
q2=[5;2;23;-4;];
quat1= utl_array2quaternion(q1);
quat2= utl_array2quaternion(q2);
quat3= quat2*quat1;
q3= utl_quat_cross_mult(q1,q2);
quat3_test=utl_array2quaternion(q3);
assert(norm(quat3-quat3_test)<=1E-12,'quaternion mutiply is broken')

v=[3;50;-23];
q= q1/norm(q1);
quat= utl_array2quaternion(q);
rotv= rotateframe(quat,v')';
rotv_test= utl_rotateframe(q,v);
assert(norm(rotv-rotv_test)<=1E-12,'quaternion rotation is broken')

rotationMatrix = rotmat(quat,'frame');
q_test= utl_dcm2quat(rotationMatrix);
assert(norm(q-q_test)<=1E-12,'dcm2quat is broken')

%special cases for vect_rot2quat
r1= [1;0;0;];
r2= [1;0;0;];
q= utl_vect_rot2quat(r1,r2);
r1_test=utl_rotateframe(q,r2);
assert(abs(norm(q)-1)<= 1E-8,'vect_rot2quat is broken')
assert(norm(r1-r1_test)<=1E-8,'vect_rot2quat is broken')
r1= [-1;0;0;];
r2= [1;0;0;];
q= utl_vect_rot2quat(r1,r2);
r1_test=utl_rotateframe(q,r2);
assert(abs(norm(q)-1)<= 1E-8,'vect_rot2quat is broken')
assert(norm(r1-r1_test)<=1E-8,'vect_rot2quat is broken')
r1= [-NaN;0;0;];
r2= [1;0;0;];
q= utl_vect_rot2quat(r1,r2);
r1_test=utl_rotateframe(q,r2);
assert(all(~isfinite(q)),'vect_rot2quat is broken')
assert(all(~isfinite(q)),'vect_rot2quat is broken')
%random tests
N=100;
for i= 1:N
    r1= randn(3,1);
    r2= randn(3,1);
    r1=r1/norm(r1);
    r2=r2/norm(r2);
    q= utl_vect_rot2quat(r1,r2);
    r1_test=utl_rotateframe(q,r2);
    assert(abs(norm(q)-1)<= 1E-8,'vect_rot2quat is broken')
    assert(norm(r1-r1_test)<=1E-8,'vect_rot2quat is broken')
end


