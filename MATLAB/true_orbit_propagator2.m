function [rf, vf, orb_elems0, orb_elemsf] = true_orbit_propagator2(r,r_ecef,v,current_time,delta_time)
%INPUT r, v in ECI coordinates
%OUTPUTS r, v in ECI coordinates at time = current_time + delta_time

global const

%calculate orbital elements with original position and velocity
orb_elems0 = posvel2kepler(r,v);

%calculate drag force
[F_envdrag, A] = env_atmospheric_drag(r,r_ecef,v); 

%calculate solar pressure acceleration
acc_solrad = solradpressure(r,const.rp_earth,A,const.MASS); %returns acceleration in ECI

%secular perturbations from Moon (third body in circular orbit)
acc_tbmoon = -const.mu_moon*(((r-const.rp_earth_moon)/norm(r-const.rp_earth_moon)^3) - (const.rp_earth_moon/norm(const.rp_earth_moon)^3));
%returns acceleration in ECI

%secular perturbations from Sun (third body in circular orbit)
acc_tbsun = -const.mu_sun*(((r-const.rp_earth)/norm(r-const.rp_earth)^3) - (const.rp_earth/norm(const.rp_earth)^3));
%returns acceleration in ECI

% perturbations due to J-coefficients
acc_Js = gravitysphericalharmonic(r, 'EGM2008',10);
%returns acceleration in ECI

%to call ODE2
state0 = zeros([6,1]);
state0(1:3)= r;
state0(4:6)= v;

statef = utl_ode2(@state_dot,[current_time,(current_time + delta_time)],state0);

    function statef = state_dot(t,y)
        %y = [x; y; z; xdot; ydot; zdot;q1;q2;q3;q4;ratex;ratey;ratez;
        %     1  2  3  4     5     6    7  8  9  10 11    12    13    
        %    
        %       fuel_ang_momentx;fuel_ang_momenty;fuel_ang_momentz;]
        %       14               15               16
        %
        % Don't have any super expensive calculation in here.
        %   Instead make first or zero order approximations of
        %   perturbations before, and pass in as a function of time.

        statef = zeros([6,1]);
        statef(1:3)=y(4:6);
        
        statef(4:6)= F_envdrag/(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun + acc_Js; 
    end

rf = statef(1:3); %new position 
vf = statef(4:6); %new velocity

%calculate new orbital elements based on new position and velocity
orb_elemsf = posvel2kepler(rf,vf);

end

function orb_elems = posvel2kepler(r,v) 

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

global const

%ensure that the inputs are column vectors
r = r(:);
v = v(:);
r_mag  = norm(r);
v_mag  = norm(v);

%calculate a using Energy
Energy = (v_mag^2/2)-(const.mu/r_mag);
a = -const.mu/(2*Energy)   ; %semi-major axis

h = cross(r,v);
h_mag = norm(h);

n = cross([0;0;1],h);
n_mag = norm(n);

e_vec = cross((v./const.mu),h)-(r./r_mag);
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

T = (2*pi/(sqrt(const.mu)))*a^(3/2); %orbital period

M = (E - e*sinE); %mean anomally
mean_motion = (2*pi)/T; %mean motion
tp = -M/mean_motion; %time of periapsis passage

%we'd like the time of periapsis passage to strictly positive
if tp < 0
    tp = T+tp;
end

orb_elems = [a,e,E,I,M,omega,Omega,T,tp,mean_motion]; %package all orbital elements

end