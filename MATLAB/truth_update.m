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
truth.time= double(truth.mission_time)*1E-9;
truth=orbit_attitude_update_ode2(truth,actuators,dt);
truth.r=truth.position_eci;
truth.v=truth.velocity_eci;

% Update position, velocity, and time
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

end