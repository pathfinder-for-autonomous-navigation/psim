function truth_update

global truth const actuators

%{

Updates the truth model of the simulation.

Global variables treated as inputs:
 * const.dt
 * const.dt__mu
 * truth.mission_time
 * truth.r
 * truth.v

Global variables treated as outputs:
 * truth.mission_time
 * truth.r
 * truth.v
 * truth.a
 * truth.e
 * truth.i
 * truth.O
 * truth.o
 * truth.nu

%}

% Numerical integration step
dt = double(const.dt) * 1e-9;
t0 = double(truth.mission_time) * 1e-9;
%y = utl_ode2(@frhs, [t0, t0 + dt], [truth.r; truth.v]);
truth.time= double(truth.mission_time)*1E-9;
truth=orbit_attitude_update_ode2(truth,actuators,dt);
truth.r=truth.position_eci;
truth.v=truth.velocity_eci;

% Update position, velocity, and time
% truth.r = transpose(y(2, 1:3));
% truth.v = transpose(y(2, 4:6));
% truth.position_eci= truth.r;
% truth.velocity_eci= truth.v;
truth.mission_time = truth.mission_time + const.dt;
truth.time= double(truth.mission_time)*1E-9;
% Update orbital elements
[   truth.a,...
    truth.e,...
    truth.i,...
    truth.O,...
    truth.o,...
    truth.nu,...
    ~,...
    ~,...
    ~,...
    ~,...
] = utl_rv2orb(truth.r, truth.v, const.mu);

function dy = frhs(~, y)
% y = [x; y; z; xdot; ydot; zdot]
% dy = [xdot; ydot; zdot; xdotdot; ydotdot; zdotdot]
% x = y(1); xdot = y(4); 
% y = y(2); ydot = y(5); 
% z = y(3); zdot = y(6);
    %[gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
    dy = zeros(6, 1);
    dy(1:3, 1) = y(4:6, 1);
    dy(4:6, 1) = env_gravity(0,y(1:3));%[gx; gy; gz];
end
end

% % attitude dynamics
% function dq = frot(t, q)
% % q = [q1; q2; q3; q1dot; q2dot; q3dot]
% % dq = [q1dot; q2dot; q3dot; q1dotdot; d2dotdot; q3dotdot]
% % q1 = q(1); q1dot = q(4)
% % q2 = q(2); q2dot = q(5)
% % q3 = q(3); q3dot = q(6) 
%         
%     Hw1 = const.Iw*(const.w1*q(4:6) + truth.ws1)*const.w1;    
%     dq(1:3) = q(4:6);
%     dq(4:6) = inv(const.I)*(tau_ext - Lw - cross(q(4:6),const.I*q(4:6) + Hw));
% end
