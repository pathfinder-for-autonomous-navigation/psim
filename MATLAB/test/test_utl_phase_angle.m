r1=[1;0;0;];
v1=[0;1;0;];
r2=[1;0;0;];

%zero phase cases
p= utl_phase_angle(r1,v1,r2);
assert(p==0,'phase angle is broken')

r2= [1;0;1000;];
p= utl_phase_angle(r1,v1,r2);
assert(p==0,'phase angle is broken')

%90 degree cases
r2= [0;1;0;];
p= utl_phase_angle(r1,v1,r2);
assert(p==pi/2,'phase angle is broken')

r2= [0;1;10;];
p= utl_phase_angle(r1,v1,r2);
assert(p==pi/2,'phase angle is broken')

r2= [0;-100;0;];
p= utl_phase_angle(r1,v1,r2);
assert(p==-pi/2,'phase angle is broken')

%180 degree edge
r2= [-1;0;0;];
p= utl_phase_angle(r1,v1,r2);
assert(p==pi || p==-pi,'phase angle is broken')
r2= [-1;1e-9;0;];
p= utl_phase_angle(r1,v1,r2);
assert(p==pi-1e-9,'phase angle is broken')
r2= [-1;-1e-9;0;];
p= utl_phase_angle(r1,v1,r2);
assert(p==-pi+1e-9,'phase angle is broken')