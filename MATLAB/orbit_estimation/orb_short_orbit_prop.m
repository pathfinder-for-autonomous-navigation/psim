function [r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] = orb_short_orbit_prop(r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,start_time)
%orb_short_orbit_prop Updates orbit, and gets the jacbian.
%   note the target is relative just to avoid floating pointing errors, for
%   the jacobians, they are as if the target is completely seperate.
%   The orbit propigator is designed for nearly circular low earth orbit, and
%   ignores all forces except gravity from env_gravity.
%
%   The jacobians are calculated assuming point mass earth.
%   
%   r_ecef(matrix (3,1)):
%       position in ecef coordinate system (m)
%   v_ecef(matrix (3,1)):
%       velocity in ecef coordinate system (m/s)
%   jacobian(matrix (6,6)):
%       Approximate jacobian of the tranform from [r_ecef;v_ecef;] input to
%       [r_ecef;v_ecef;] output.
%   rel_target_r_ecef(matrix (3,1)):
%       position of the target relative to r_ecef in ecef coordinate system (m)
%   rel_target_v_ecef(matrix (3,1)):
%       velocity of the target relative to v_ecef in ecef coordinate system (m/s)
%   target_jacobian(matrix (6,6)):
%       Approximate jacobian of the tranform from [rel_target_r_ecef+r_ecef;rel_target_v_ecef+v_ecef;] input to
%       [rel_target_r_ecef+r_ecef;rel_target_v_ecef+v_ecef;] output.
%   dt(double -0.2 to 0.2):
%       Time step, how much time to update the orbit (s)
%   start_time(double): seconds since const.INITGPS_WN
%
% Started by Nathan Zimmerberg on Jan 29, 2020
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 29, 2020
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const

%% step 1a ecef->ecef0
r_ecef0= r_ecef;
v_ecef0= v_ecef+cross(const.earth_rate_ecef,r_ecef);
rel_target_r_ecef0= rel_target_r_ecef;
rel_target_v_ecef0= rel_target_v_ecef+cross(const.earth_rate_ecef,rel_target_r_ecef);
%% step 1b get circular orbit reference
energy= 0.5*dot(v_ecef0,v_ecef0)-const.mu/norm(r_ecef0);
a=-const.mu/2/energy;
h_ecef0= cross(r_ecef0,v_ecef0);
x= r_ecef0;
x=x/norm(x)*a;
y= cross(h_ecef0,r_ecef0);
y=y/norm(y)*a;
omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
[orb_r,orb_v]= circular_orbit(0,0,omega,x,y);
rel_r= r_ecef0-orb_r;
rel_v= v_ecef0-orb_v;
%% step 2 drift
rel_r= rel_r+rel_v*dt*0.5;
rel_target_r_ecef0= rel_target_r_ecef0+rel_target_v_ecef0*dt*0.5;
%% step 3a calc acceleration at the half step
t= 0.5*dt;
[orb_r,~]= circular_orbit(t,0,omega,x,y);
r_ecef0= rel_r+orb_r;
target_pos_ecef0= r_ecef0+rel_target_r_ecef0;
r_half_ecef0= r_ecef0;
target_pos_half_ecef0= target_pos_ecef0;
now = start_time + t; %current time in seconds
dcm_ecef_ecef0 = relative_earth_dcm(t);
pos_ecef=dcm_ecef_ecef0*r_ecef0;
target_pos_ecef=dcm_ecef_ecef0*target_pos_ecef0;
% perturbations due to J-coefficients; returns acceleration in ECEF
g_ecef=env_gravity(now,pos_ecef);
target_g_ecef=env_gravity(now,target_pos_ecef)-g_ecef;
%convert to ECEF0
acc =dcm_ecef_ecef0'*g_ecef+const.mu*orb_r/a^3;
target_acc =dcm_ecef_ecef0'*target_g_ecef;
%% step 3b kick velocity
rel_v= rel_v + acc*dt*1;
rel_target_v_ecef0= rel_target_v_ecef0+target_acc*dt;
%% step 4 drift
rel_r= rel_r+rel_v*dt*0.5;
rel_target_r_ecef0= rel_target_r_ecef0+rel_target_v_ecef0*dt*0.5;
%% step 5a get back absolute orbit
[orb_r,orb_v]= circular_orbit(dt,0,omega,x,y);
r_ecef0= rel_r+orb_r;
v_ecef0= rel_v+orb_v;
%% step 5b rotate back to ecef
dcm_ecef_ecef0 = relative_earth_dcm(dt);
r_ecef= dcm_ecef_ecef0*r_ecef0;
v_ecef= dcm_ecef_ecef0*v_ecef0;
rel_target_r_ecef= dcm_ecef_ecef0*rel_target_r_ecef0;
rel_target_v_ecef= dcm_ecef_ecef0*rel_target_v_ecef0;
%% step 6 remove cross r term from velocity
v_ecef= v_ecef-cross(const.earth_rate_ecef,r_ecef);
rel_target_v_ecef= rel_target_v_ecef-cross(const.earth_rate_ecef,rel_target_r_ecef);

%% compute jacobians
jacobian= get_jacobian(r_half_ecef0,dcm_ecef_ecef0,dt);
target_jacobian= get_jacobian(target_pos_half_ecef0,dcm_ecef_ecef0,dt);
end

function [r,v]= circular_orbit(t,t0,omega,x,y)
    %Returns the position from circular orbit at time t in ecef0
    theta= (t-t0)*norm(omega);
    r= x*cos(theta)+y*sin(theta);
    v= cross(omega,r);
end

function [A_EI] = relative_earth_dcm(dt)
    %relative_earth_dcm returns the dcm to rotate from initial ecef to ecef
    %dt seconds latter.
    global const
    earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
    earth_omega= norm(const.earth_rate_ecef);
    theta= earth_omega*dt;% earth rotation angle
    A_EI= cos(theta)*eye(3)-sin(theta)*cross_product_matrix(earth_axis)+(1-cos(theta))*(earth_axis*earth_axis');
end

function M = cross_product_matrix(x)
    %cross_product_matrix 
    % see eq 2.55 in the ADC book
    M=[ 0 -x(3) x(2);
        x(3) 0 -x(1);
        -x(2) x(1) 0;];
end

function H = hessian_helper(x)
    %returns the hessian of gravity, mks units
    %from equation (3.154) in Fundamentals of Spacecraft Attitude Determination and Control
    global const
    r= norm(x);
    H= -const.mu/r^3*(eye(3)-3*(x*x')/r^2);
end

function [J]= get_jacobian(r_half_ecef0,dcm_ecef_ecef0,dt)
    %returns the jacobians
    %target_r_half_ecef0 and r_half_ecef0 are to position vetors on the
    %half step mks units, dt is the time step.
    %dcm_ecef_ecef0 is the dcm to rotate from initial ecef to ecef
    %dt seconds latter.
    global const
    wx= cross_product_matrix(const.earth_rate_ecef);
    H= hessian_helper(r_half_ecef0);
    A= dcm_ecef_ecef0;
    Hdt= H*dt;
    I= eye(3);
    iwx= I+dt*0.5*wx;
    hiwx= Hdt*iwx;
    rr41= I+dt*wx+hiwx*dt*0.5;
    rv41= dt*I+Hdt*dt*dt*0.25;
    vr41= hiwx+wx;
    vv41= I+Hdt*dt*0.5;
    wxtA= wx'*A;
    J= [A*rr41 A*rv41;
        wxtA*rr41+A*vr41 A*vv41+wxtA*rv41;];
end