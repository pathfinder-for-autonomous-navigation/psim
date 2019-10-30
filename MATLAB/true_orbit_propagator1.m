function [r, v] = true_orbit_propagator(r,v,current_time,delta_time)
%INPUT r, v in ECI coordinates
%OUTPUTS r, v in ECI coordinates at time = current_time + delta_time

global const

%calculate orbital elements with original position and velocity
[a0,e0,E0,I0,omega0,Omega0,T0,tp0] = posvel2kepler(r,v,const.mu);
n0 = sqrt(const.mu/a0^3); %mean motion

%secular perturbations due to atmopsheric drag
[~,rho_p,Cd,A,h_scale]  = env_atmospheric_drag(time,r,v);
Q = 1- (2*const.earth_rate_ecef*(1-e0)^1.5/(n0*sqrt(1+e0)))*cos(I0);
del_a_rev_drag = -2*pi*Q*A*Cd*a0^2*const.MASS^-1*rho_p*(I0 + ...
    2*e0*I1 + 0.75*e0^2*(I0+I2) +0.25*e0^3*(3*I1+I3))*exp(-a0*e0/h_scale); 
del_e_rev_drag = -2*pi*Q*A*Cd*a*const.MASS^-1*rho_p*(I1 + (e0/2)*(I0+I2)-...
    (e0^2/8)*(5*I1-I3)+(e0^3/16)*(5*I0+4*I2-I4))*exp(-a0*e0/h_scale);

%calculate modified Bessel Functions of the First Kind
I = zeros(4,z);
for nu = 0:3
    I(nu+1,:) = besseli(nu,z);
end

% I1 = (1/pi)*integral(exp(zcos(theta))*cos(theta),0,pi) - ...
%     (sin(pi)/pi)*integral(exp(-z*cosh(t)-t),0, Inf);
% I2 = (1/pi)*integral(exp(zcos(theta))*cos(2*theta),0,pi) - ...
%     (sin(2*pi)/pi)*integral(exp(-z*cosh(t)-2*t),0, Inf);
% I3 = (1/pi)*integral(exp(zcos(theta))*cos(3*theta),0,pi) - ...
%     (sin(3*pi)/pi)*integral(exp(-z*cosh(t)-3*t),0, Inf);
% I4 = (1/pi)*integral(exp(zcos(theta))*cos(4*theta),0,pi) - ...
%     (sin(4*pi)/pi)*integral(exp(-z*cosh(t)-4*t),0, Inf);
% %what is z????

%call solar pressure force
acc_solrad = AccelSolrad(r,r_Earth,r_Moon,r_Sun,r_SunSSB,Area,mass,Cr,P0,AU,shm);

%secular perturbations from Moon (third body in circular orbit)
acc_tbmoon = -const.mu_moon*(((r-const.rp_earth_moon)/norm(r-const.rp_earth_moon)^3) - (const.rp_earth_moon/norm(const.rp_earth_moon)^3));
Omega_dot_sec_moon = (-3/16)*const.mu_moon*(2+3*e0^2)*(2-3*sin(I3)^2)*cos(I0)/(n0*norm(const.rp_earth_moon)^3*sqrt(1-e0^2));
omega_dot_sec_moon = (3/16)*const.mu_moon*(2-3*sin(I3)^2)*(e0^2+4-5*sin(I0)^2)/(n0*norm(const.rp_earth_moon)^3*sqrt(1-e0^2));

%secular perturbations from Sun (third body in circular orbit)
acc_tbsun = -const.mu_sun*(((r-const.rp_earth)/norm(r-const.rp_earth)^3) - (const.rp_earth/norm(const.rp_earth)^3));
Omega_dot_sec_sun = (-3/16)*const.mu_sun*(2+3*e0^2)*(2-3*sin(I3)^2)*cos(I0)/(n0*norm(const.rp_earth)^3*sqrt(1-e0^2));
omega_dot_sec_sun = (3/16)*const.mu_sun*(2-3*sin(I3)^2)*(e0^2+4-5*sin(I0)^2)/(n0*norm(const.rp_earth)^3*sqrt(1-e0^2));

%secular perturbations due to J2 if not using perturbing potential of full model
%placeholder
J2 = 1082.6E-6; %for Earth
Omega_dot_sec_J2 = (-3*n0*const.R_EARTH^2*J2)*cos(I0)/(2*a0^2*(1-e0^2)^2);
omega_dot_sec_J2 = 1.5*J2*n0*(const.R_EARTH/(a0*(1-e0^2)))^2*(2-2.5*sin(I0)^2);
Mdot_sec_J2 = n0-((3*J2*n0*const.R_EARTH^2)/(4*a0^2*(1-e0^2)^1.5))*(2-3*sin(I0)^2);

% perturbations due to J2 zonal harmonic (most significant
% model is EGM2008 --> planet coords in ECEF coordinates
% three different models we can use and compare
%ode45 to get final vectors or just use J2?
%[gx gy, gz] = gravityzonal(planetCoords, 'Earth', 2); %gravity values in the x,y,z directions of ECEF coord system
%[gx gy gz] = gravitysphericalharmonic(planetCoords, 'EGM2008','EGM2008');
%R = earth_perturb_pot(r,theta,phi);

%calculate new orbital elements
a_new = a0 + (del_a_rev_drag)*(delta_time/T0);
e_new = e0 + (del_e_rev_drag)*(delta_time/T0);
I_new = I0;
Omega_new = Omega0 + (Omega_dot_sec_moon + Omega_dot_sec_sun + Omega_dot_sec_J2)*(delta_time); 
omega_new = omega0 + (omega_dot_sec_moon + omega_dot_sec_sun + omega_dot_sec_J2)*(delta_time);
M_new = M0 + Mdot_sec_J2;

%calculate position and velocity based on new orbital elements
[t_new,x_new,y_new,E_new] = newton_2body(a_new,e_new,const.mu);

end

function [t,x,y,E] = newton_2body(a,e,mu) 
% Two-body numerical integration
% Inputs:
%   a - semi-major axis
%   e - eccentricity
%   mu - gravitational parameter (with same distance units as a)
%
% Output:
%   t - 1000 element time array between 0 and the orbital period
%   x,y - Orbital radius components for one orbit in the perifocal
%         frame (e,q) directions. Same size as t.
%   E - Specific orbital energy over one orbital period. Same
%       size as t.

h = sqrt(a*mu*(1-e^2)) ; %angular momentum; do we put in magnitude or vector form? 
Tp = (2*pi/sqrt(mu))*a^(3/2) ; %orbital period

t = linspace(0,Tp,1e3).'; %time array

%initial conditions:
x0 = a*(1-e); %r at rps
y0 = 0; %planet only along e direction
r0 = sqrt(x0^2 + y0^2); %need to plug in for vy0 
vx0 = 0 ; 
vy0 = sqrt(mu*((2/r0)-(1/a))) ;

z0 = [x0,y0,vx0,vy0].';
%integrate
[~,z] = ode45(@newtongravity2d,t,z0);

x = z(:,1); %set the final position
y = z(:,2); %set the final position
r = sqrt(x.^2 + y.^2); %magnitude of position
v = sqrt(z(:,3).^2+z(:,4).^2); %magnitude of velocity vector
E = (v.^2/2) - (mu./r);

%r = sqrt((rx)^2 + (ry)^2)%magnitude of position; radius
%rdot = sqrt((rxdot)^2 + (rydot)^2); %magnitude of velocity

    %integrator function
    function dz = newtongravity2d(~,z)
        %Input: state vector z
        %output: state derivative dz
        
        x1 = z(1); y1 = z(2);
        r = sqrt((x1^2)+(y1^2));
        rxddot= -mu*x1/r^3;
        ryddot= -mu*y1/r^3; 
        dz = [z(3) z(4) rxddot ryddot]';
    end
end

function [a,e,E,I,omega,Omega,T,tp] = posvel2kepler(r,v,mu) 

%Convert orbital position and velocity to orbital elements
%Inputs:
%   r - 3x1 array: orbital position vector
%   v - 3x1 array: orbital velocity vector
%   mu - scalar: gravitational parameter
%
%Outputs:
%   a - scalar: semi-major axis
%   e - scalar: eccentricity
%   E - scalar: eccentric anomaly (in radians)
%   I - scalar: inclination (in radians)
%   omega - scalar: argument of periapsis (in radians)
%   Omega - scalar: longitude of the ascending node (in radians)
%   T - scalar: orbital period
%   tp - scalar: time of periapse passage

%ensure that the inputs are column vectors
r = r(:);
v = v(:);
r_mag  = norm(r);
v_mag  = norm(v);

%calculate a using Energy
Energy = (v_mag^2/2)-(mu/r_mag);
a = -mu/(2*Energy)   ; %semi-major axis

h = cross(r,v);
h_mag = norm(h);

n = cross([0;0;1],h);
n_mag = norm(n);

e_vec = cross((v./mu),h)-(r./r_mag);
e = norm(e_vec); %eccentricity

cosE = (1/e)*(1-(r_mag/a)) ; %cosine of the eccentric anomaly (from radius eq)
true_anom = acos((a*cosE-a*e)/r_mag);
sinE = (sin(true_anom)*sqrt(1-e^2))/(1+e*cos(true_anom)); %sine of eccentric anomaly (from velocity eq.)
E = mod(atan2(sinE,cosE),2*pi); %ecentric anomaly

I = acos(dot((h./h_mag),[0;0;1]))  ; %inclination
I = mod(I,2*pi); %inclination must be in proper range

omega = acos(dot(n,e_vec)/(n_mag*e))  ; %argument of periapsis
omega = 2*pi - mod(omega,2*pi); %omega 

Omega = acos(dot([1;0;0],(n./n_mag)))    ; %longitude of ascending node
Omega = 2*pi - mod(Omega,2*pi);

T = (2*pi/(sqrt(mu)))*a^(3/2); %orbital period

M = (E - e*sinE);
mean_motion = (2*pi)/T;
tp = -M/mean_motion; %time of periapsis passage

%we'd like the time of periapsis passage to strictly positive
if tp < 0
    tp = T+tp;
end

end