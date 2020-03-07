%%%special cases
A = utl_area([0,0,-1]');
assert((A-0.01) <= 1E-10,'util_area is broken')

A = utl_area([0,0,1]');
assert((A-0.01) <= 1E-10,'util_area is broken')

A = utl_area([1,0,0]');
assert((A-0.03) <= 1E-10,'util_area is broken')

A = utl_area([-1,0,0]');
assert((A-0.03) <= 1E-10,'util_area is broken')

A = utl_area([0,1,0]');
assert((A-0.03) <= 1E-10,'util_area is broken')

A = utl_area([0,-1,0]');
assert((A-0.03) <= 1E-10,'util_area is broken')

A = utl_area([sqrt(2)/2,-sqrt(2)/2,0]');
assert((A-0.0424) <= 1E-4,'util_area is broken')

A = utl_area([sqrt(3)/3,-sqrt(3)/3,sqrt(3)/3]');
A_test = 0.3*0.1*sqrt(3)/3 + 0.3*0.1*sqrt(3)/3 + 0.1*0.1*sqrt(3)/3;
assert((A-A_test) <= 1E-4,'util_area is broken')

%%%random cases
for k = 1:100
    [A_test,v] = test_area();
    A = utl_area(v');
    assert((A_test-A) <= 1E-4,'util_area is broken')
end

function [A_test,v] = test_area()
s = randn(3,1);
v = s/norm(s);

c0 = [0,0,0]'; c1 = [.1,0,0]'; c2 = [0,.1,0]'; c3 = [0 0 -0.3]';
unit1 = (c1-c0); unit2 = (c2-c0); unit3 = (c3-c0);

proj1 = unit1 - (dot(unit1,v)*v);
proj2 = unit2 - (dot(unit2,v)*v);
proj3 = unit3 - (dot(unit3,v)*v);

area1 = norm(cross(proj1,proj2));
area2 = norm(cross(proj1,proj3));
area3 = norm(cross(proj2,proj3));

A_test = area1+area2+area3;
end