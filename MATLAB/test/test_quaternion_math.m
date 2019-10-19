config();
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
