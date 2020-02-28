function [r_final,v_final] = true_orbit_propagator2(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECEF coordinates in m and m/s
    % start time of mission; pan time in continuous seconds past const.INITGPS_WN
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECEF coordinates in m and m/s at time = start_time + duration

    global const

    %package initial state to call ODE45
    state0 = zeros([6,1]);
    %get v in psuedo intertial frame.
    state0(1:3)= r;
    state0(4:6)= v+cross(const.earth_rate_ecef,r);
    [quat_ecef0_eci,~]=env_earth_attitude(start_time);
    
    %create tarray
    opts = odeset('RelTol', 1E-12, 'AbsTol', 1E-4);%, %'OutputFcn','odephas3');
    tspan = [0,duration];
    [t_array, states] = ode113(@state_dot, tspan, state0, opts, perturbs, start_time);
    %statef = utl_ode2(@state_dot,[current_time,(current_time + delta_time)],state0);
    r_ecef0= states(end,1:3)';
    v_ecef0= states(end,4:6)';
    earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
    theta= norm(const.earth_rate_ecef)*duration;% earth rotation angle
    quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
    
    r_ecef= utl_rotateframe(quat_ecef_ecef0,r_ecef0);
    v_ecef= utl_rotateframe(quat_ecef_ecef0,v_ecef0);
    r_final= r_ecef;
    v_final= v_ecef-cross(const.earth_rate_ecef,r_ecef);

    function statef = state_dot(t, state0, perturbs, start_time)
        %y = [x; y; z; xdot; ydot; zdot]
        %fprintf('%f \n',t)
        
        earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
        theta= norm(const.earth_rate_ecef)*t;% earth rotation angle
        quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
        quat_ecef0_ecef= utl_quat_conj(quat_ecef_ecef0);
        
        now = start_time + t; %current time in seconds
        
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
            rp_earth_moon = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth_moon'); %positional vector from Moon to Earth; used for 3rd body perturb calcs
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
            rp_earth = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth'); %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
            acc_tbsun = -const.mu_sun*(((r+rp_earth)/norm(r+rp_earth)^3) - (rp_earth/norm(rp_earth)^3)); 
        else
            acc_tbsun = zeros(3,1);
        end

        % gravitational force in ECI
        now = start_time + t; %current time in seconds
        pos_ecef0=r;
        pos_ecef=utl_rotateframe(quat_ecef_ecef0,pos_ecef0);
        % perturbations due to J-coefficients; returns acceleration in ECEF
        g_ecef=env_gravity(now,pos_ecef);
        %convert to ECEF0
        acc_Js =utl_rotateframe(quat_ecef0_ecef,g_ecef);        

        statef = zeros([6,1]);
        statef(1:3)=state0(4:6);
        
        %acceration in m/s^2, ECEF0 coords
        statef(4:6) = acc_Js + F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun;
    end
end