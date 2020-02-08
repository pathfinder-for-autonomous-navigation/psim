function [t_array, states, orb_elemsf] = true_orbit_propagator(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECI coordinates in m and m/s
    % start time of mission; gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECI coordinates in m and m/s at time = current_time + delta_time

    global const

    %package initial state to call ODE45
    state0 = zeros([6,1]);
    state0(1:3)= r;
    state0(4:6)= v;
    
    
    %create tarray
    opts = odeset('RelTol', 1E-12, 'AbsTol', 1E-4, 'OutputFcn','odephas3');
    tspan = [0,duration];
    [t_array, states] = ode113(@state_dot, tspan, state0, opts, perturbs, start_time);
    %statef = utl_ode2(@state_dot,[current_time,(current_time + delta_time)],state0);
    
    orb_elemsf = zeros(length(t_array)); 
    for count = 1:length(t_array)
        rf  = states(count,1:3)'; %new position 
        vf = states(count,4:6)'; %new velocity 
        %quatf = states(count,7:10); %new quaternions
        
        %calculate orbital elements
        [a, eMag, i, O, o, nu, truLon, argLat, lonPer, p] = utl_rv2orb(rf, vf, const.mu);
        orb_elemsf(count,1) = a*1E3; orb_elemsf(count,2) = eMag; orb_elemsf(count,3) = i;
        orb_elemsf(count,4) = O; orb_elemsf(count,5) = o; orb_elemsf(count,6) = nu;
        orb_elemsf(count,7) = truLon; orb_elemsf(count,8) = argLat; orb_elemsf(count,9) = lonPer;
        orb_elemsf(count,10) = p*1E3;
    end
end

function statef = state_dot(t, state0, perturbs, start_time)
        %y = [x; y; z; xdot; ydot; zdot]
        fprintf('%f \n',t)
        
        global const
        
        % unpack
        r = state0(1:3);  v = state0(4:6);
        
        %if user wants to investigate affect of only certain perturbing forces
        
        %calculate drag force
        if perturbs.drag == 1
            F_envdrag = env_atmospheric_drag(t, r,v);
        else
            F_envdrag = zeros(3,1);
        end
        
        %calculate solar pressure acceleration
        if perturbs.solrad == 1
            acc_solrad = env_solradpressure(r,const.rp_earth,const.MASS); %returns acceleration in ECI
        else
            acc_solrad = zeros(3,1);
        end
        
        %secular perturbations from Moon (third body in circular orbit)
        %returns acceleration in ECI
        if perturbs.bodmoon == 1
            now = start_time + t; %current time in seconds
            datetime = utl_time2datetime(now,const.INITGPS_WN); 
            [rp_earth_moon,~] = planetEphemeris(juliandate(datetime),'Moon','Earth','421');
            rp_earth_moon = 1E3*rp_earth_moon'; %positional vector from Moon to Earth; used for 3rd body perturb calcs
            acc_tbmoon = -const.mu_moon*(((r+rp_earth_moon)/norm(r+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
        else
            acc_tbmoon = zeros(3,1);
        end
        
        %secular perturbations from Sun (third body in circular orbit)
        %returns acceleration in ECI
        if perturbs.bodsun == 1
            now = start_time + t; %current time in seconds
            datetime = utl_time2datetime(now,const.INITGPS_WN); 
            [rp_earth,~] = planetEphemeris(juliandate(datetime),'Sun','Earth','421');
            rp_earth = 1E3*rp_earth; %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
            acc_tbsun = -const.mu_sun*(((r+rp_earth')/norm(r+rp_earth')^3) - (rp_earth'/norm(rp_earth')^3)); 
        else
            acc_tbsun = zeros(3,1);
        end

        % gravitational force in ECI
        now = start_time + t; %current time in seconds
        [quat_ecef_eci,~]=env_earth_attitude(now);
        quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
        pos_eci=r;
        pos_ecef=utl_rotateframe(quat_ecef_eci,pos_eci);
        %[g_ecef,~,G_ecef]= env_gravity(t,pos_ecef);
        %g_eci=utl_rotateframe(quat_eci_ecef,g_ecef);
        g_ecef= zeros(3,1);
        % perturbations due to J-coefficients; returns acceleration in ECEF
        %[g_ecef(1),g_ecef(2),g_ecef(3)]  = gravitysphericalharmonic(pos_ecef', 'EGM96',perturbs.numJs);
        g_ecef=env_gravity(now,pos_ecef);
        %convert to ECI
        acc_Js =utl_rotateframe(quat_eci_ecef,g_ecef);        

        statef = zeros([6,1]);
        statef(1:3)=state0(4:6);
        
        %acceration in m/s^2, ECI coords
        statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun + acc_Js;
        %statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun;
        
        
end