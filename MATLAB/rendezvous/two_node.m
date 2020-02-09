config();
a  = 6793137.0;
n = utl_orbrate(a);
%  a : semimajor axis (m)
%  n : orbital rate (rad / s)
[phi_rrab, phi_rvab, phi_vrab, phi_vvab] = utl_cw(pi/2/n, n);
phi_ab= [phi_rrab, phi_rvab;
    phi_vrab,phi_vvab];
[phi_rrba, phi_rvba, phi_vrba, phi_vvba] = utl_cw(3*pi/2/n, n);
phi_ba= [phi_rrba, phi_rvba;
    phi_vrba,phi_vvba];

%IC
maxdv_init= 0.1;
num_drift_orbits= 24;
x= -3/n*maxdv_init;
y= -6*pi/n*num_drift_orbits;
xdot= 2*maxdv_init;
ydot= -3*maxdv_init;

orbits= 2000;
max_dv_thrust= 5E-3;
xs=zeros(orbits,1);
xdots=zeros(orbits,1);
ys=zeros(orbits,1);
ydots=zeros(orbits,1);
y_dvs=zeros(orbits,1);
for i= 1:orbits
    %execute manuver at point B
    x_dv=-xdot;
    x_dv= max(min(x_dv,max_dv_thrust),-max_dv_thrust);
    xdot= xdot+x_dv;
    p=1.5E-6;
    d=100E-6;
    %   the system is  x(k+1)= x(k)+v(k)+u(k)
    y_x= y;
    y_v= (-12*pi*x-1/n*6*pi*ydot);
    y_maxu= 1/n*6*pi*max_dv_thrust*0.8;
    y_u= discrete_suicide_burn(y_maxu, y_v,y_x);
    y_dv= y_u/(-1/n*6*pi);
    y_dv= max(min(y_dv,max_dv_thrust),-max_dv_thrust);
    y_dvs(i)=y_dv;
    ydot= ydot+y_dv;
    % propagate to point A
    state= [x;y;0;xdot;ydot;0];
    state= phi_ba*state;
    x= state(1);
    y= state(2);
    xdot= state(4);
    ydot= state(5);
    %execute manuver at point A
    x_dv= -n*(4*x+1/n*xdot+2/n*ydot);
    x_dv= max(min(x_dv,max_dv_thrust),-max_dv_thrust);
    xdot= xdot+x_dv;
    % propagate to point B
    state= [x;y;0;xdot;ydot;0];
    state= phi_ab*state;
    x= state(1);
    y= state(2);
    xdot= state(4);
    ydot= state(5);
    xs(i)=x;
    ys(i)=y;
    xdots(i)=xdot;
    ydots(i)=ydot;
end
figure;
plot(ys);
figure;
plot(y_dvs/max_dv_thrust);
figure;
plot(xs);
figure;
plot(xdots);

function u= discrete_suicide_burn(maxu, v,x)
%impements a discrete suicide burn 
%   calculates the required control u, to drive x and v to 0 asap.
%   the system is  x(k+1)= x(k)+v(k)+u(k)
%zero case
if(x==0 && v==0)
    "stoped"
    u=0;
    return;
end
%make x negative
signx= -sign(x);
x= x*signx;
v= v*signx;
if (v<=0)
    "moving away"
    u= -x-v;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%build min distance to stop.
kmin= floor(v/maxu);
residv= mod(v,maxu);
area= -x;
areamin= (kmin-1)*(kmin)/2*maxu+residv*kmin;
area= area-areamin;%residual area
maxdarea= kmin*(maxu-residv);
if (maxdarea>=area)
    "step 1"
    %implement step 1
    u=area/kmin-maxu;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%step 2, add step
kmin= kmin+1;
area= area-maxdarea;%residual area
maxdarea= kmin*maxu;
if (maxdarea>=area)
    "step 2"
    %implement step 2
    u=area/kmin-residv;
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
%step 3, add step
kmin= kmin+1;
area= area-maxdarea;%residual area
maxdarea= kmin*residv;
if (maxdarea>=area)
    "step 3"
    %partial speed ahead
    %implement step 3
    u=area/kmin+(maxu-residv);
    u= min(max(u,-maxu),maxu);
    u= u*signx;
    return
end
"full speed ahead"
u= maxu;%full speed ahead
u= u*signx;
end


