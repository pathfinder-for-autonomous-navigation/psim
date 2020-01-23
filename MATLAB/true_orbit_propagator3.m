function [r_final,v_final] = true_orbit_propagator2(r,v,start_time,duration, perturbs)
    % INPUT r, v in ECEF coordinates in m and m/s
    % start time of mission; gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
    % duration is mission duration you want to look at; in seconds
    % OUTPUTS r, v in ECEF coordinates in m and m/s at time = current_time + duration

    global const
    global count

    r_ecef0= r;
    v_ecef0= v+cross(const.earth_rate_ecef,r);
    [~,pot_energy,~]= env_gravity(start_time,r_ecef0);
    %pot_energy= const.mu/norm(r_ecef0);
    energy= 0.5*dot(v_ecef0,v_ecef0)-const.mu/norm(r_ecef0);
    a=-const.mu/2/energy;
    [quat_ecef0_eci,~]=env_earth_attitude(start_time);
    h_ecef0= cross(r_ecef0,v_ecef0);
    x= r_ecef0;
    x=x/norm(x)*a;
    y= cross(h_ecef0,r_ecef0);
    y=y/norm(y)*a;
    omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
    t0=0;

    

    
    %high order integrators
    %https://doi.org/10.1016/0375-9601(90)90092-3
    %https://pdf.sciencedirectassets.com/271541/1-s2.0-S0375960100X05075/1-s2.0-0375960190900923/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEHIaCXVzLWVhc3QtMSJGMEQCIF1ECb38EYCxayPQRI0hpkbfjJc8eQ%2FAlwjL8FEtSVN1AiAg%2BNndqK38eCFIGiNiIh1DajL%2FBlIVrQFa6idjCf4l9Cq9Awj7%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAIaDDA1OTAwMzU0Njg2NSIMsEfyy6bWwA0G0iHLKpEDA%2FBHuVZ%2BVrJnY6l8heiEZ7jxtjL2zdPELqgIjgiJUo50nM%2FMAOinJx82WVIbukNl56aGWSCe6Pysdn2mS9cQx%2BrF2HAoTs%2BpiBadw%2FuAtggU4RQLc9%2Ffu4by%2FaCZYOYkTV7dA8KYAibKr76LaKZmZcal%2B2iHVc6pZom5iof3O%2FIMV%2BYiKuxkGKv9aiztBwn59PsE0v4odCDkbfs%2FNeIzNNjvZOqMd84xXSKB%2FND4TJArkhpBBo%2F9DShQp4wBwZepjKHaDAlmSJbyXt1nvlkr2c55HSXYMFzA0HocbScbekInnxoVRRIBhSlAJqVKLeTKScO%2B3%2BEuIJM9i3Mdf%2FZF4LfNdQad1Edalc6Xg7seo1Cm13aX92SIbBql8JIHQrhL%2F8YO4CNgtv6f4ajHqprlsq4Dbd6hVeM%2F%2F9pO7cyBibdRZMFDFKj%2Bn1yNx6voOQna4fXd9953vNwVer4TbZEMuxAI3gyte7V9uxc4qqi33wYavVFpBeHv8%2FAp1u5pE0h4NvB7SCTJ2VFEl7m6mKN5mIQwsLSZ8QU67AFWLmQMZ2r2Bx6Eos9Sx7Q9MQju0X5Dacz5rMWai7MeWLv4cbY05t%2BFzppKCGLTEE%2B80u6E59onOWV1ppmvgYYD%2BH%2FxMskaYil9mZuhhfSkrVbTEbUmyWIURFu76ht72ZGzfLiuw35ilknpQoeewBNIGwvEn%2FWbHsv1vrmn0JZk2TNVZBDaS7jEXpJINs7%2BJKo%2F82ngZKaGLwMLH%2BDUrbdd%2B0WqjiKX6y%2BAbBVpLjlZsRj%2BPoV2d97UR6ABtYSVHv%2FWITdO7Wd930KQQizMrh3yZDDDUrj4UQAqWpqqZv38OQ2HvDHcvhvx4wowyA%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20200121T024645Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYSQKDTZ5H%2F20200121%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=087c847babff243b457807c797f220e3743de6d63902d0943a61cd30e30ee95b&hash=4a586ea181fec35778b5f7e60a625cdd055f00d9ea16572cef4a41192adaebc9&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=0375960190900923&tid=spdf-306995ff-861f-40bd-9c97-a080a5d8c1b2&sid=0f7242a41192874f6a6a8bb-6ac9f383dac9gxrqa&type=client
    w4=[ 1/(2-2^(1/3)) ];
    w8a= [-0.161582374150097E1,
        -0.244699182370524E1,
        -0.716989419708120E-2,
        0.244002732616735E1,
        0.157739928123617E0,
        0.182020630970714E1,
        0.104242620869991E1,];
    w8b= [-0.169248587770116E-2,
        0.289195744315849E1,
        0.378039588360192E-2,
        -0.289688250328827E1,
        0.289105148970595E1,
        -0.233864815101035E1,
        0.148819229202922E1,];
    w8c= [0.311790812418427,
        -0.155946803821447E1,
        -0.167896928259640E1,
        0.166335809963315E1,
        -0.106458714789183E1,
        0.136934946416871E1,
        0.629030650210433E0,];
    w8d= [0.102799849391985E0,
        -0.196061023297549E1,
        0.193813913762276E1,
        -0.158240635368243E0,
        -0.144485223686048E1,
        0.253693336566229E0,
        0.914844246229740E0];
    w8e= [0.227738840094906E-1,
        0.252778927322839E1,
        -0.719180053552772E-1,
        0.536018921307285E-2,
        -0.204809795887393E1,
        0.107990467703699E0,
        0.130300165760014E1,];
        
        
    w6a= [-0.117767998417887E1,
        0.235573213359357E0,
        0.784513610477560E0,];
    w6b= [-0.213228522200144E1,
        0.426068187079180E-2,
        0.143984816797678E1,];
    w6c= [0.152886228424922E-2,
        -0.214403531630539E1,
        0.144778256239930E1,];
    w=w6a;
    m=length(w);
    N= round(duration/15/(m*2+1));
    w0= 1-2*sum(w);
    c=zeros(1,2*m+2);
    d=zeros(1,2*m+1);
    for i= 2:m
        c(i)= 0.5*(w(m+2-i)+w(m+1-i));
        c(2*m+3-i)= 0.5*(w(m+2-i)+w(m+1-i));
    end
    c(1)= 0.5*w(m);
    c(2*m+2)= 0.5*w(m);
    c(m+1)= 0.5*(w(1)+w0);
    c(m+2)= 0.5*(w(1)+w0);
    for i= 1:m
        d(i)= w(m+1-i);
        d(2*m+2-i)= w(m+1-i);
    end
    d(m+1)=w0;
    dt= duration/(N);
    [orb_r,orb_v]= circular_orbit(0,t0,omega,x,y);
    rel_r= r_ecef0-orb_r;
    rel_v= v_ecef0-orb_v;
    
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
        for j= 1:length(d)
            rel_r= rel_r+rel_v*dt*c(j);
            t= t+dt*c(j);
            [orb_r,~]= circular_orbit(t,t0,omega,x,y);
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
            if perturbs.bodsun == 1
                acc_tbsun = -const.mu_sun*(((orb_r+rel_r+rp_earth)/norm(orb_r+rel_r+rp_earth)^3) - (rp_earth/norm(rp_earth)^3));
                acc= acc+acc_tbsun;
            end
            if perturbs.bodmoon == 1
                acc_tbmoon = -const.mu_moon*(((orb_r+rel_r+rp_earth_moon)/norm(orb_r+rel_r+rp_earth_moon)^3) - (rp_earth_moon/norm(rp_earth_moon)^3)); 
                acc= acc+acc_tbmoon;
            end
            rel_v= rel_v + acc*dt*d(j);
        end
        rel_r= rel_r+rel_v*dt*c(end);
        %reset orbit
        [r_ecef0,v_ecef0]= circular_orbit(i*dt,t0,omega,x,y);
        r_ecef0= rel_r+r_ecef0;
        v_ecef0= rel_v+v_ecef0;
        energy= 0.5*dot(v_ecef0,v_ecef0)-const.mu/norm(r_ecef0);
        a=-const.mu/2/energy;
        h_ecef0= cross(r_ecef0,v_ecef0);
        x= r_ecef0;
        x=x/norm(x)*a;
        y= cross(h_ecef0,r_ecef0);
        y=y/norm(y)*a;
        omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
        t0=i*dt;
        [orb_r,orb_v]= circular_orbit(t0,t0,omega,x,y);
        rel_r= r_ecef0-orb_r;
        rel_v= v_ecef0-orb_v;
    end

    [r_ecef0,v_ecef0]= circular_orbit(duration,t0,omega,x,y);
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

function [r,v]= circular_orbit(t,t0,omega,x,y)
    %Returns the position from circular orbit at time t in ecef0
    theta= (t-t0)*norm(omega);
    r= x*cos(theta)+y*sin(theta);
    v= cross(omega,r);
end