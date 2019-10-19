function q = utl_dcm2quat(a)
% finds the quaternion representation of a direction cosine matrix a
%copied directly from Attitude Determination Using Two Vector Measurements
%1999 by Landis Markley
%https://www.researchgate.net/publication/4706531_Attitude_Determination_Using_Two_Vector_Measurements

% find maximum of trace or diagonal element of direction cosine matrix
tra    = trace(a);
[mx,i] = max([a(1,1) a(2,2) a(3,3) tra]);

% compute unnormalized quaternion
if i==1, q = [2*mx+1-tra;a(1,2)+a(2,1);a(1,3)+a(3,1);a(2,3)-a(3,2)];
elseif i==2, q = [a(2,1)+a(1,2);2*mx+1-tra;a(2,3)+a(3,2);a(3,1)-a(1,3)];
elseif i==3, q = [a(3,1)+a(1,3);a(3,2)+a(2,3);2*mx+1-tra;a(1,2)-a(2,1)];
else, q = [a(2,3)-a(3,2);a(3,1)-a(1,3);a(1,2)-a(2,1);1+tra];      end

% normalize the quaternion
q = q/norm(q);

