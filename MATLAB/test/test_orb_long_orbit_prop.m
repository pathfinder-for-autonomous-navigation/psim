%load GRACE data; in m, m/s, ECEF
rng(2,'threefry');

global const

trials= 20;
for trial= 1:trials
    %duration of the mission you want to simulate in seconds
    if trial< trials/4 
        duration = 24*60*60*(rand()-0.5)*2; % 1 day
    else
        duration = 1.5*60*60*(rand()-0.5)*2; % approximately 1 orbit
    end

    a  = 6.371E6+300E3+500E3*rand();  % Semimajor axis                        (m)
    e  = 0.02*rand();      % Eccentricity                          (unitless)
    i  = pi*rand();  % Inclination angle                     (rad)
    O  = 2*pi*rand();        % Right ascension of the ascending node (rad)
    o  = 2*pi*rand();        % Argument of perigee                   (rad)
    nu = 2*pi*rand();   % True anamoly                          (rad)

    [   r,...  % Position (m)   [eci]
        v,...  % Velocity (m/s) [eci]
    ] = utl_orb2rv(a*(1-e*e), e, i, O, o, nu, const.mu);
    startr_ecef= r;
    startv_ecef= v-cross(const.earth_rate_ecef,r);

    starttime = rand()*(5*365*24*60*60);

    perturbs.drag = 0;
    perturbs.solrad = 0;
    perturbs.bodmoon = 0;
    perturbs.bodsun = 0;
    perturbs.numJs = 10;

    [r_final,v_final]  = true_orbit_propagator2(startr_ecef, startv_ecef, starttime, duration, perturbs);
    [r_ecef,v_ecef] = orb_long_orbit_prop(startr_ecef,startv_ecef,duration,starttime);
    assert(norm(r_final-r_ecef)<1E-4*abs(duration),"position too far off from reference integrator "+norm(r_final-r_ecef) + " on test: "+trial+ " duration "+duration);
    assert(norm(v_final-v_ecef)<1E-7*abs(duration),"velocity too far off from reference integrator "+norm(v_final-v_ecef) + " on test: "+trial+ " duration "+duration);
end

[r_ecef,v_ecef] = orb_long_orbit_prop(startr_ecef,startv_ecef,0,starttime);
assert(norm(r_ecef-startr_ecef)==0,"0 edge case failed");
assert(norm(v_ecef-startv_ecef)==0,"0 edge case failed");