function [r_final,v_final] = true_orbit_propagator2(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECEF coordinates in m and m/s
    % start time of mission; gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECEF coordinates in m and m/s at time = current_time + duration

    global const
    global count
    count=0;

    r_ecef0= r;
    v_ecef0= v+cross(const.earth_rate_ecef,r);
    %[~,pot_energy,~]= env_gravity(start_time,r_ecef0);
    pot_energy= const.mu/norm(r_ecef0);
    energy= 0.5*dot(v_ecef0,v_ecef0)-pot_energy;
    a=-const.mu/2/energy;
    [quat_ecef0_eci,~]=env_earth_attitude(start_time);
    h_ecef0= cross(r_ecef0,v_ecef0);
    x= r_ecef0;
    x=x/norm(x)*a;
    y= cross(h_ecef0,r_ecef0);
    y=y/norm(y)*a;
    omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
    function [r,v]= circular_orbit(t)
        %Returns the position from circular orbit at time t in ecef0
        theta= t*norm(omega);
        r= x*cos(theta)+y*sin(theta);
        v= cross(omega,r);
    end

    
    function acc= acc_rel(t,rel_r)
        [orb_r,orb_v]= circular_orbit(t);
        now = start_time + t; %current time in seconds
        earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
        theta= norm(const.earth_rate_ecef)*t;% earth rotation angle
        quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
        quat_ecef0_ecef= utl_quat_conj(quat_ecef_ecef0);
        pos_ecef=utl_rotateframe(quat_ecef_ecef0,orb_r+rel_r);%
        % perturbations due to J-coefficients; returns acceleration in ECEF
        g_ecef=env_gravity(now,pos_ecef);
        count=count+1;
        %convert to ECEF0
        acc =utl_rotateframe(quat_ecef0_ecef,g_ecef)+const.mu*orb_r/a^3;
    end
    function stated = state_dot(t, state)
        state1= zeros(6,1);
        stated= zeros(6,1);
        state1(1:6)=state(1:6);
        stated(1:3)= state1(4:6);
        stated(4:6)= acc_rel(t,state1(1:3));
    end
    
   
    [orb_r,orb_v]= circular_orbit(0);
    rel_r= r_ecef0-orb_r;
    rel_v= v_ecef0-orb_v;
    state0 = [rel_r;rel_v;];
    opts = odeset('RelTol', 1E-12, 'AbsTol', 1E-3);%, %'OutputFcn','odephas3');
    tspan = [0,duration];
    [~, states] = ode45(@state_dot, tspan, state0, opts);
    %statef = utl_ode2(@state_dot,[current_time,(current_time + delta_time)],state0);
    rel_r= states(end,1:3)';
    rel_v= states(end,4:6)';
    [r_ecef0,v_ecef0]= circular_orbit(duration);
    r_ecef0= rel_r+r_ecef0;
    v_ecef0= rel_v+v_ecef0;
    earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
    theta= norm(const.earth_rate_ecef)*duration;% earth rotation angle
    quat_ecef_ecef0= [earth_axis*sin(theta/2);cos(theta/2);];
    
    r_ecef= utl_rotateframe(quat_ecef_ecef0,r_ecef0);
    v_ecef= utl_rotateframe(quat_ecef_ecef0,v_ecef0);
    r_final= r_ecef;
    v_final= v_ecef-cross(const.earth_rate_ecef,r_ecef);
end