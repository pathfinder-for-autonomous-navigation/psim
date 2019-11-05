function [t_array, states, orb_elemsf] = true_orbit_propagator(r,v,current_time,delta_time)
    %INPUT r, v in ECI coordinates
    %OUTPUTS r, v in ECI coordinates at time = current_time + delta_time

    global const
    
    %intialize for initial gravitational acceleration
    %[quat_ecef_eci_initial,~]=env_earth_attitude(current_time);
    %[quat_ecef_eci_final,~]=env_earth_attitude(current_time+delta_time);
    %quat_ecef_eci_initial= utl_array2quaternion(quat_ecef_eci_initial);
    %quat_ecef_eci_final= utl_array2quaternion(quat_ecef_eci_final);

    %package initial state to call ODE45
    state0 = zeros([6,1]);
    state0(1:3)= r;
    state0(4:6)= v;

    %create tarray
    t_array= linspace(current_time, (current_time + delta_time), 10000);

    [t_array states] = ode45(@state_dot, t_array, state0);
    %statef = utl_ode2(@state_dot,[current_time,(current_time + delta_time)],state0);
    
    orb_elemsf = zeros(length(t_array)); 
    for count = 1:length(t_array)
        rf  = states(count,1:3)'; %new position 
        vf = states(count,4:6)'; %new velocity 
        
        %calculate orbital elements
        [a, eMag, i, O, o, nu, truLon, argLat, lonPer, p] = utl_rv2orb(rf, vf, const.mu);
        orb_elemsf(count,1) = a; orb_elemsf(count,2) = eMag; orb_elemsf(count,3) = i;
        orb_elemsf(count,4) = O; orb_elemsf(count,5) = o; orb_elemsf(count,6) = nu;
        orb_elemsf(count,7) = truLon; orb_elemsf(count,8) = argLat; orb_elemsf(count,9) = lonPer;
        orb_elemsf(count,10) = p;
    end
end

function statef = state_dot(t_array, state0)
        %y = [x; y; z; xdot; ydot; zdot]
        
        global const
        
        % unpack
        r = state0(1:3);  v = state0(4:6);
        
%         quat_ecef_eci= earth_first_order_rotation(t_array, current_time, delta_time);
%         quat_eci_ecef= conj(quat_ecef_eci);
%         pos_eci= v;
%         pos_ecef=rotateframe(quat_ecef_eci,pos_eci')';
%         [g_ecef,~,~]= env_gravity(current_time,pos_ecef);
%         g_eci=rotateframe(quat_eci_ecef,g_ecef')';
    
        %calculate drag force
        [F_envdrag, A] = env_atmospheric_drag(t_array, r,v); 

        %calculate solar pressure acceleration
        acc_solrad = solradpressure(r,const.rp_earth,A,const.MASS); %returns acceleration in ECI

        %secular perturbations from Moon (third body in circular orbit)
        acc_tbmoon = -const.mu_moon*(((r-const.rp_earth_moon)/norm(r-const.rp_earth_moon)^3) - (const.rp_earth_moon/norm(const.rp_earth_moon)^3));
        %returns acceleration in ECI

        %secular perturbations from Sun (third body in circular orbit)
        acc_tbsun = -const.mu_sun*(((r-const.rp_earth')/norm(r-const.rp_earth')^3) - (const.rp_earth'/norm(const.rp_earth')^3));
        %returns acceleration in ECI

        % perturbations due to J-coefficients
        acc_Js = gravitysphericalharmonic(r', 'EGM2008',10);
        %returns acceleration in ECI

        statef = zeros([6,1]);
        statef(1:3)=state0(4:6);
        
        statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun + acc_Js;
        %statef(4:6) = F_envdrag./(const.MASS) + acc_solrad + acc_tbmoon + acc_tbsun;
end

function quat_ecef_eci= earth_first_order_rotation(t, current_time, delta_time)
    %returns earths attitude at time t, using a first order
    %approximation (slerp)
    %of the rotation from the start to end of delta_time
    %disp((t-initial_state.time)/delta_time)
    T=min((t-current_time)/delta_time,1.0);
    quat_ecef_eci= slerp(quat_ecef_eci_initial,quat_ecef_eci_final,T);
end