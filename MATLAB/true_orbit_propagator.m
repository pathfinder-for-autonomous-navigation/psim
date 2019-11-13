function [t_array, states, orb_elemsf] = true_orbit_propagator(r,v,start_time,duration)
    %INPUT r, v in ECI coordinates in m and m/s
    %OUTPUTS r, v in ECI coordinates in m and m/s at time = current_time + delta_time

    global const
    

    %package initial state to call ODE45
    state0 = zeros([6,1]);
    state0(1:3)= r;
    state0(4:6)= v;
    
%     %intialize for initial gravitational acceleration
%     [quat_ecef_eci_initial,~]=env_earth_attitude(current_time);
%     [quat_ecef_eci_final,~]=env_earth_attitude(current_time+delta_time);
%     quat_ecef_eci_initial= utl_array2quaternion(quat_ecef_eci_initial);
%     quat_ecef_eci_final= utl_array2quaternion(quat_ecef_eci_final);
%     state0(7:10) = quat_body_eci
    
    
    %create tarray
    opts = odeset('RelTol', 1E-12, 'AbsTol', 1E-3);
    [t_array, states] = ode113(@state_dot, [start_time, (start_time + duration)], state0, opts);
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

function statef = state_dot(t, state0)
        %y = [x; y; z; xdot; ydot; zdot]
        
        global const
        
        % unpack
        r = state0(1:3);  v = state0(4:6);
    
        %calculate drag force
        [F_envdrag, A] = env_atmospheric_drag(t, r,v); 

        %calculate solar pressure acceleration
        acc_solrad = env_solradpressure(r,const.rp_earth,A,const.MASS); %returns acceleration in ECI
        
        %use perihelion date instead of t_array?
        datetime = utl_time2datetimeUTC(t,0); %second input should be init_GPS_week_number(int): initial GPS week number.  
        
        [rp_earth_moon,~] = planetEphemeris(juliandate(datetime),'Moon','Earth');
        rp_earth_moon = 1E3*rp_earth_moon'; %positional vector from Moon to Earth; used for 3rd body perturb calcs
        [rp_earth,~] = planetEphemeris(juliandate(datetime),'Sun','Earth');
        rp_earth = 1E3*rp_earth; %positional vector from Sun to Earth; used for 3rd body perturb and solar radiation pressure calcs
    

        %secular perturbations from Moon (third body in circular orbit)
        acc_tbmoon = -const.mu_moon*(((r-rp_earth_moon)/norm(r-rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3));
        %returns acceleration in ECI

        %secular perturbations from Sun (third body in circular orbit)
        acc_tbsun = -const.mu_sun*(((r-rp_earth')/norm(r-rp_earth')^3) - (rp_earth'/norm(rp_earth')^3));
        %returns acceleration in ECI

        % perturbations due to J-coefficients
        %returns acceleration in ECI
        acc_Js = gravitysphericalharmonic(r', 'EGM2008',10);
        
%         % gravitational force in ECI
%         quat_ecef_eci= earth_first_order_rotation(t_array);
%         quat_eci_ecef= conj(quat_ecef_eci);
%         quat_body_eci=utl_array2quaternion(state0(7:10));
%         quat_eci_body= conj(quat_body_eci);
%         quat_body_ecef=quat_eci_ecef*quat_body_eci;
%         pos_eci=state0(1:3);
%         pos_ecef=rotateframe(quat_ecef_eci,pos_eci')';
%         [g_ecef,~,G_ecef]= env_gravity(t_array,pos_ecef);
%         g_eci=rotateframe(quat_eci_ecef,g_ecef')';

        statef = zeros([6,1]);
        statef(1:3)=state0(4:6);
        
        
        %acceration in m/s^2, ECI coords
        statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun + acc_Js;
        %statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun;
        
        %statef(7:10) = 
        
        
end

function quat_ecef_eci= earth_first_order_rotation(t, current_time, delta_time)
    %returns earths attitude at time t, using a first order
    %approximation (slerp)
    %of the rotation from the start to end of delta_time
    %disp((t-initial_state.time)/delta_time)
    T=min((t-current_time)/delta_time,1.0);
    quat_ecef_eci= slerp(quat_ecef_eci_initial,quat_ecef_eci_final,T);
end