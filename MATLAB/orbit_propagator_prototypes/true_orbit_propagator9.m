function [r_final,v_final] = true_orbit_propagator7(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECEF coordinates in m and m/s
    % start time of mission; gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECEF coordinates in m and m/s at time = current_time + duration

    global const
    global count
    


    N= max(1,round(duration/0.2));
    dt= duration/(N);
    %secular perturbations from Moon (third body in circular orbit)
    %returns acceleration in ECEF0
    if perturbs.bodmoon == 1
        [quat_ecef0_eci,~]=env_earth_attitude(start_time);
        times= ((1:N)-0.5)*dt+start_time;
        jdates = juliandate(utl_time2datetime(times,const.INITGPS_WN)); 
        [rp_earth_moons_eci,~] = planetEphemeris(jdates','Moon','Earth','421');
        %rp_earth_moon = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth_moon'); %positional vector from Moon to Earth; used for 3rd body perturb calcs
        %acc_tbmoon = -const.mu_moon*(((r+rp_earth_moon)/norm(r+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
    end

    %secular perturbations from Sun (third body in circular orbit)
    %returns acceleration in ECEF0
    if perturbs.bodsun == 1
        [quat_ecef0_eci,~]=env_earth_attitude(start_time);
        times= ((1:N)-0.5)*dt+start_time;
        jdates = juliandate(utl_time2datetime(times,const.INITGPS_WN)); 
        [rp_earths_eci,~] = planetEphemeris(jdates','Sun','Earth','421');
        %rp_earth = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth'); %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
        %acc_tbsun = -const.mu_sun*(((r+rp_earth)/norm(r+rp_earth)^3) - (rp_earth/norm(rp_earth)^3)); 
    end
    
    for i=1:N
        r_ecef0= r;
        v_ecef0= v+cross(const.earth_rate_ecef,r);
        energy= 0.5*dot(v_ecef0,v_ecef0)-const.mu/norm(r_ecef0);
        a=-const.mu/2/energy;
        [quat_ecef0_eci,~]=env_earth_attitude(start_time);
        h_ecef0= cross(r_ecef0,v_ecef0);
        x= r_ecef0;
        x=x/norm(x)*a;
        y= cross(h_ecef0,r_ecef0);
        y=y/norm(y)*a;
        omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
        [orb_r,orb_v]= circular_orbit(0,0,omega,x,y);
        rel_r= r_ecef0-orb_r;
        rel_v= v_ecef0-orb_v;
        %secular perturbations from Moon (third body in circular orbit)
        %returns acceleration in ECEF0
        if perturbs.bodmoon == 1
            rp_earth_moon = utl_rotateframe(quat_ecef0_eci,1E3*rp_earth_moons_eci(i,:)');
        end
        
        %secular perturbations from Sun (third body in circular orbit)
        %returns acceleration in ECEF0
        if perturbs.bodsun == 1
            rp_earth = utl_rotateframe(quat_ecef0_eci,1E3*rp_earths_eci(i,:)'); %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
        end
        rel_r= rel_r+rel_v*dt*0.5;
        t= 0.5*dt;
        [orb_r,~]= circular_orbit(t,0,omega,x,y);
        r_ecef0= rel_r+orb_r;
        now = start_time + (i-0.5)*dt; %current time in seconds
        earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
        theta= norm(const.earth_rate_ecef)*t;% earth rotation angle
        quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
        quat_ecef0_ecef= utl_quat_conj(quat_ecef_ecef0);
        pos_ecef=utl_rotateframe(quat_ecef_ecef0,r_ecef0);%
        % perturbations due to J-coefficients; returns acceleration in ECEF
        g_ecef=env_gravity(now,pos_ecef);
        count=count+1;
        %convert to ECEF0
        acc =utl_rotateframe(quat_ecef0_ecef,g_ecef)+const.mu*orb_r/a^3;
        if perturbs.bodsun == 1
            acc_tbsun = -const.mu_sun*(((r_ecef0+rp_earth)/norm(r_ecef0+rp_earth)^3) - (rp_earth/norm(rp_earth)^3));
            acc= acc+acc_tbsun;
        end
        if perturbs.bodmoon == 1
            acc_tbmoon = -const.mu_moon*(((r_ecef0+rp_earth_moon)/norm(r_ecef0+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
            acc= acc+acc_tbmoon;
        end
        rel_v= rel_v + acc*dt*1;
        rel_r= rel_r+rel_v*dt*0.5;
        [orb_r,orb_v]= circular_orbit(dt,0,omega,x,y);
        r_ecef0= rel_r+orb_r;
        v_ecef0= rel_v+orb_v;
        earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
        theta= norm(const.earth_rate_ecef)*dt;% earth rotation angle
        quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
        r= utl_rotateframe(quat_ecef_ecef0,r_ecef0);
        v= utl_rotateframe(quat_ecef_ecef0,v_ecef0);
        v= v-cross(const.earth_rate_ecef,r);
    end
    r_final= r;
    v_final= v;
end

function [r,v]= circular_orbit(t,t0,omega,x,y)
    %Returns the position from circular orbit at time t in ecef0
    theta= (t-t0)*norm(omega);
    r= x*cos(theta)+y*sin(theta);
    v= cross(omega,r);
end