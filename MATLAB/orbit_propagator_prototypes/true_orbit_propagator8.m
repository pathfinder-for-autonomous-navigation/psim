function [r_final,v_final] = true_orbit_propagator8(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECEF coordinates in m and m/s
    % start time of mission; gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECEF coordinates in m and m/s at time = current_time + duration

    global const
    global count

    N= max(1,round(duration/0.1));
    dt= duration/(N);
    %secular perturbations from Moon (third body in circular orbit)
    %returns acceleration in ECEF0
    if perturbs.bodmoon == 1
        times= ((1:N)-0.5)*dt+start_time;
        jdates = juliandate(utl_time2datetime(times,const.INITGPS_WN)); 
        [rp_earth_moons_eci,~] = planetEphemeris(jdates','Moon','Earth','421');
        %rp_earth_moon = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth_moon'); %positional vector from Moon to Earth; used for 3rd body perturb calcs
        %acc_tbmoon = -const.mu_moon*(((r+rp_earth_moon)/norm(r+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
    end

    %secular perturbations from Sun (third body in circular orbit)
    %returns acceleration in ECEF0
    if perturbs.bodsun == 1
        times= ((1:N)-0.5)*dt+start_time;
        jdates = juliandate(utl_time2datetime(times,const.INITGPS_WN)); 
        [rp_earths_eci,~] = planetEphemeris(jdates','Sun','Earth','421');
        %rp_earth = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth'); %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
        %acc_tbsun = -const.mu_sun*(((r+rp_earth)/norm(r+rp_earth)^3) - (rp_earth/norm(rp_earth)^3)); 
    end
    
    for i=1:N
        t= (i-1)*dt;
        r= r + v*dt*0.5;
        %secular perturbations from Moon (third body in circular orbit)
        %returns acceleration in ECEF0
        if perturbs.bodmoon == 1
            [quat_ecef_eci,~]=env_earth_attitude(start_time+t);
            rp_earth_moon = utl_rotateframe(quat_ecef_eci,1E3*rp_earth_moons_eci(i,:)');
        end
        
        %secular perturbations from Sun (third body in circular orbit)
        %returns acceleration in ECEF0
        if perturbs.bodsun == 1
            [quat_ecef_eci,~]=env_earth_attitude(start_time+t);
            rp_earth = utl_rotateframe(quat_ecef_eci,1E3*rp_earths_eci(i,:)'); %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
        end
        now = start_time + t; %current time in seconds
        % perturbations due to J-coefficients; returns acceleration in ECEF
        g_ecef=env_gravity(now,r);
        count=count+1;
        %convert to ECEF0
        acc =g_ecef-2*cross(const.earth_rate_ecef,v)-cross(const.earth_rate_ecef,cross(const.earth_rate_ecef,r));
        if perturbs.bodsun == 1
            acc_tbsun = -const.mu_sun*(((r+rp_earth)/norm(r+rp_earth)^3) - (rp_earth/norm(rp_earth)^3));
            acc= acc+acc_tbsun;
        end
        if perturbs.bodmoon == 1
            acc_tbmoon = -const.mu_moon*(((r+rp_earth_moon)/norm(r+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
            acc= acc+acc_tbmoon;
        end
        v= v + acc*dt;
        r= r + v*dt*0.5;
    end
    r_final= r;
    v_final= v;
end