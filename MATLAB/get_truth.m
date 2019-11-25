function value = get_truth(name,dynamics)
% get_truth Retrieved name info about the satellite given string name and the dynamics of a satellite.
%   use get_truth('help') to see docs
global const
if startsWith(name,"help",'IgnoreCase',true)
    value= {
'Retrieved name info about the satellite given string name and the dynamics of a satellite.'
'For example, get_truth(''magnetic field body'',main_state.follower.dynamics) returns the real magnetic field in the body frame.'
' '
'get_truth(''magnetic field eci'',main_state.follower.dynamics) returns the real magnetic field in the eci frame.'
'get_truth(''quat body eci'',main_state.follower.dynamics) returns the quaternion that rotates vectors from eci to body'
'get_truth(''dcm body eci'',main_state.follower.dynamics) returns the Direction Cosine Matrix(DCM) that rotates vectors from eci to body'
'get_truth(''rate body eci body'',main_state.follower.dynamics) returns the angular rate of the transform from eci to body in the body frame.'
'get_truth(''rate body'',main_state.follower.dynamics) short hand for ''rate body eci body'' returns the angular rate of the transform from eci to body in the body frame.'
'get_truth(''rate body eci eci'',main_state.follower.dynamics) returns the angular rate of the transform from eci to body in the eci frame.'
'get_truth(''velocity eci'',main_state.follower.dynamics) returns the velocity of the center of mass of the sat relative to earth in the eci frame.'
'get_truth(''velocity ecef'',main_state.follower.dynamics) velocity measurements take into account additional velocity from rate cross radius.'
'returns the velocity of the center of mass of the sat relative to earth in the ecef frame.'
' '
'get_truth has full support for the following frames and associated coordinate systems:'
'    ''body'': The +x face has the antennas, the -z face has the docking magnets.'
'    ''eci'': Earth Centered Inertial, an inertial frame, z axis is close to(but not exactly) the north pole.'
'    ''ecef'': Earth Centered Earth Fixed, rotates with earth, z axis is the north pole.'
'    ''lvlh'': Local Vertical Local Horizontal, x axis is position from earth to sat, z axis is orbit normal'
'    ''vbn'': Velocity Binormal Normal, x axis is sats eci velocity, z axis is orbit normal.'
'    Additional frames and associated coordinate systems can be added as needed, just modify quaternion_from_string(frame1,frame2) and rate_from_string(frame) nested functions in get_truth.'
' '
'get_truth supports the following vectors, these have a frame and associated coordinate system:'
'    ''rate'': angular rate (rad/s)'
'    ''velocity'': velocity of the center of mass of the sat relative to earth (m/s)'
'    ''position'': position of the center of mass of the sat relative to earth (m)'
'    ''sat2sun'': the normalized vector from the satellite to the sun (unitless)'
'    ''magnetic field'': (T)'
'    ''gravity'': acceleration from gravity, doesn''t account for coriolus effect (m/s^2)'
'    ''total angular momentum'': Total internal angular momentum of the sat (Nms)'
'    ''orbital angular momentum'': Orbital angular momentum of the sat (Nms)'
'    ''eccentricity vector'': Vector pointing from apoapsis to periapsis, using osculating elements with magnitude equal to the orbit''s scalar eccentricity (unitless)'
'    ''wheel rate'': x,y, and z, wheel angular rates (rad/s)'
'    ''antenna'': GPS and Quake antenna normal (unitless)'
'    ''docking face'': Docking face normal (unitless)'
' '
'get_truth also supports the following scalar values, these don''t have a frame:'
'    ''time'': Time since initial GPS week (s)'
'    ''fuel mass'': The mass of the fuel (kg)'
'    ''orbital energy'': (J)'
'    ''rotational energy'': (J)'
'    ''semimajor axis'': osculating semimajor axis (m)'
'    ''eccentricity'': osculating eccentricity (unitless)'
'    ''inclination angle'': osculating inclination angle (rad)'
'    ''right ascension of the ascending node'': osculating right ascension of the ascending node (rad)'
'    ''argument of perigee'': osculating argument of perigee (rad)'
'    ''true anomaly'': osculating true anomaly (rad)'
'    ''eclipse'': 1 if in eclipse (boolean)'
'    ''solar panel area in sun'': projected area of solar panels in sun light (m^2)'
};

elseif startsWith(name,"rate",'IgnoreCase',true)
    v=split(name);
    if length(v) == 2
        value=rate_from_string(v(end));
    elseif length(v) == 4
        frame1= v(end-2);
        frame2= v(end-1);
        frame3= v(end);
        rate_frame1_eci_frame1=rate_from_string(frame1);
        rate_frame2_eci_frame2=rate_from_string(frame2);
        rate_frame1_eci_eci=rotate_vector_strings("eci",frame1,rate_frame1_eci_frame1);
        rate_frame2_eci_eci=rotate_vector_strings("eci",frame2,rate_frame2_eci_frame2);
        rate_frame1_frame2_eci= rate_frame1_eci_eci-rate_frame2_eci_eci;
        value= rotate_vector_strings(frame3,"eci",rate_frame1_frame2_eci);
    else
        error(" give three frames or one frame after rate")
    end 
elseif startsWith(name,"velocity",'IgnoreCase',true)
    v=split(name);
    frame=v(end);
    if strcmpi(frame,"eci")
        value= dynamics.velocity_eci;
        return
    end
    v_eci= dynamics.velocity_eci;
    r_eci= dynamics.position_eci;
    rate_frame_eci_frame= rate_from_string(frame);
    quat_frame_eci=quaternion_from_string(frame,"eci");
    r_frame= utl_rotateframe(quat_frame_eci,r_eci);
    v_frame= utl_rotateframe(quat_frame_eci,v_eci) - cross(rate_frame_eci_frame,r_frame);
    value= v_frame;
elseif startsWith(name,"position",'IgnoreCase',true)
    v=split(name);
    value= rotate_vector_strings(v(end),"eci",dynamics.position_eci);
elseif startsWith(name,"sat2sun",'IgnoreCase',true)
    S_eci= env_sun_vector(dynamics.time);
    v=split(name);
    value= rotate_vector_strings(v(end),"eci",S_eci);
elseif startsWith(name,"magnetic field",'IgnoreCase',true)
    [quat_ecef_eci,~]=env_earth_attitude(dynamics.time);
    pos_ecef=utl_rotateframe(quat_ecef_eci,dynamics.position_eci);
    B_ecef= env_magnetic_field(dynamics.time,pos_ecef);
    v=split(name);
    value= rotate_vector_strings(v(end),"ecef",B_ecef);
elseif startsWith(name,"gravity",'IgnoreCase',true)
    [quat_ecef_eci,~]=env_earth_attitude(dynamics.time);
    pos_ecef=utl_rotateframe(quat_ecef_eci,dynamics.position_eci);
    [g_ecef,~,~]= env_gravity(dynamics.time,pos_ecef);
    v=split(name);
    value= rotate_vector_strings(v(end),"ecef",g_ecef);
elseif startsWith(name,"total angular momentum",'IgnoreCase',true)
    hb_body= const.JB*dynamics.angular_rate_body+const.JWHEEL*dynamics.wheel_rate_body;
    spacecraft_angular_momentum_eci= utl_rotateframe(utl_quat_conj(dynamics.quat_body_eci),hb_body)+dynamics.fuel_net_angular_momentum_eci;
    v=split(name);
    value= rotate_vector_strings(v(end),"eci",spacecraft_angular_momentum_eci);
elseif startsWith(name,"orbital angular momentum",'IgnoreCase',true)
    orbital_angular_momentum_eci=cross(dynamics.position_eci,dynamics.velocity_eci)*(const.MASS+dynamics.fuel_mass);
    v=split(name);
    value= rotate_vector_strings(v(end),"eci",orbital_angular_momentum_eci);
elseif startsWith(name,"eccentricity vector",'IgnoreCase',true)
    v= dynamics.velocity_eci;
    r= dynamics.position_eci;
    e_eci=(v'*v/const.mu - 1/norm(r))*r-r'*v/const.mu*v;
    v= split(name);
    value= rotate_vector_strings(v(end),"eci",e_eci);
elseif startsWith(name,"wheel rate",'IgnoreCase',true)
    v=split(name);
    value= rotate_vector_strings(v(end),"body",dynamics.wheel_rate_body);
elseif startsWith(name,"antenna",'IgnoreCase',true)
    v=split(name);
    value= rotate_vector_strings(v(end),"body",[1;0;0]);
elseif startsWith(name,"docking face",'IgnoreCase',true)
    v=split(name);
    value= rotate_vector_strings(v(end),"body",[0;0;-1]);
elseif startsWith(name,"orbital energy",'IgnoreCase',true)
    [quat_ecef_eci,~]=env_earth_attitude(dynamics.time);
    [~,PG,~]= env_gravity(dynamics.time,utl_rotateframe(quat_ecef_eci,dynamics.position_eci));
    PE=-PG*(const.MASS+dynamics.fuel_mass);
    KE= 0.5*(dynamics.fuel_mass+const.MASS)*dynamics.velocity_eci'*dynamics.velocity_eci;
    value= KE+PE;
elseif startsWith(name,"rotational energy",'IgnoreCase',true)
    rotational_energy= dynamics.angular_rate_body'*const.JB*dynamics.angular_rate_body;
    rotational_energy= rotational_energy+dynamics.wheel_rate_body'*const.JWHEEL*dynamics.wheel_rate_body;
    Jfuel= const.JFUEL_NORM*dynamics.fuel_mass;
    rotational_energy= rotational_energy+dynamics.fuel_net_angular_momentum_eci'*inv(Jfuel)*dynamics.fuel_net_angular_momentum_eci;
    rotational_energy= 0.5*rotational_energy;
    value= rotational_energy;
elseif startsWith(name,"semimajor axis",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= a;
elseif startsWith(name,"eccentricity",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= eMag;
elseif startsWith(name,"inclination angle",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= i;
elseif startsWith(name,"right ascension of the ascending node",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= O;
elseif startsWith(name,"argument of perigee",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= o;
elseif startsWith(name,"true anomaly",'IgnoreCase',true)
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer,p] = utl_rv2orb(dynamics.position_eci, dynamics.velocity_eci, const.mu);
    value= nu;
elseif startsWith(name,"time",'IgnoreCase',true)
    value= dynamics.time; 
elseif startsWith(name,"fuel mass",'IgnoreCase',true)
    value= dynamics.fuel_mass; 
elseif startsWith(name,"eclipse",'IgnoreCase',true)
    value=env_eclipse(dynamics.position_eci,env_sun_vector(dynamics.time));
elseif startsWith(name,"solar panel area in sun",'IgnoreCase',true)
    if get_truth("eclipse",dynamics)
        value=0;
    else
        sat2sun_body= get_truth("sat2sun body",dynamics); 
        solar_panel_normals=[0.03 0 0;%+X
                             -0.03 0 0;%-X
                             0 0.03 0;%+Y
                             0 -0.03 0;%-Y
                             0 0 0.01;];%+Z
        solar_panel_areas_in_sun=max(solar_panel_normals*sat2sun_body,0);
        value=sum(solar_panel_areas_in_sun);
    end
elseif startsWith(name,"dcm",'IgnoreCase',true)
    v=split(name);
    q=quaternion_from_string(v(end-1),v(end));
    quat=utl_array2quaternion(q);
    value= rotmat(quat,'frame');
elseif startsWith(name,"quat",'IgnoreCase',true)
    v=split(name);
    assert(length(v)>=3,"quaternion must have two frames, destination and source")
    value=quaternion_from_string(v(end-1),v(end));
else
    error(name+" not avaliable Call `get_truth('help')` for complete documentation about supported values")
end


    function quat= quaternion_from_string(frame1,frame2)
        % returns quat_frame1_frame2
        %works right now by converting from frame 2 to eci, then from eci to frame1
        if strcmpi(frame1,frame2) %if both frames are the same just return identity
            quat=[0;0;0;1;];
            return
        end
        quat_frame1_eci=quaternion_frame_eci(frame1);
        quat_eci_frame2=utl_quat_conj(quaternion_frame_eci(frame2));
        function quat= quaternion_frame_eci(frame)
            if startsWith(frame,"body",'IgnoreCase',true)
                quat= dynamics.quat_body_eci;
            elseif startsWith(frame,"eci",'IgnoreCase',true)
                quat= [0;0;0;1;];
            elseif startsWith(frame,"ecef",'IgnoreCase',true)
                quat_ecef_eci= env_earth_attitude(dynamics.time);
                quat= quat_ecef_eci;
            elseif startsWith(frame,"lvlh",'IgnoreCase',true)
                xaxis_eci= dynamics.position_eci;
                xaxis_eci= xaxis_eci/norm(xaxis_eci);
                zaxis_eci= cross(dynamics.position_eci,dynamics.velocity_eci);
                zaxis_eci= zaxis_eci/norm(zaxis_eci);
                yaxis_eci= cross(zaxis_eci,xaxis_eci);
                dcm_lvlh_eci= [xaxis_eci';yaxis_eci';zaxis_eci'];
                quat= utl_dcm2quat(dcm_lvlh_eci);
            elseif startsWith(frame,"vbn",'IgnoreCase',true)
                xaxis_eci= dynamics.velocity_eci;
                xaxis_eci= xaxis_eci/norm(xaxis_eci);
                zaxis_eci= cross(dynamics.position_eci,dynamics.velocity_eci);
                zaxis_eci= zaxis_eci/norm(zaxis_eci);
                yaxis_eci= cross(zaxis_eci,xaxis_eci);
                dcm_vbn_eci= [xaxis_eci';yaxis_eci';zaxis_eci'];
                quat= utl_dcm2quat(dcm_vbn_eci);
            else
                error(frame+" frame not avaliable")
            end
        end
        quat= utl_quat_cross_mult(quat_frame1_eci,quat_eci_frame2);
        %normalize quat just to be safe
        quat= quat/norm(quat);
    end

    function rate_frame_eci_frame=rate_from_string(frame)
        if startsWith(frame,"body",'IgnoreCase',true)
            rate_frame_eci_frame= dynamics.angular_rate_body;
        elseif startsWith(frame,"eci",'IgnoreCase',true)
            rate_frame_eci_frame= zeros(3,1);
        elseif startsWith(frame,"ecef",'IgnoreCase',true)
            rate_frame_eci_frame= const.earth_rate_ecef;
        elseif startsWith(frame,"lvlh",'IgnoreCase',true)
            error("lvlh rate not implemented yet")
        elseif startsWith(frame,"vbn",'IgnoreCase',true)
            error("vbn rate not implemented yet")
        else
            error(frame+" frame not avaliable")
        end
    end
    
    function u_frame= rotate_vector_strings(frame,origin_frame,u)
        if strcmpi(frame,origin_frame) %if both frames are the same just return identity
            u_frame=u;
            return
        end
        q_frame_origin_frame= quaternion_from_string(frame,origin_frame);
        u_frame= utl_rotateframe(q_frame_origin_frame,u);
    end
end