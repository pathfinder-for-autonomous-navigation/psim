function [r_ecef,v_ecef] = orb_long_orbit_prop(r_ecef,v_ecef,duration,start_time)
%orb_short_orbit_prop Updates orbit
%   The orbit propigator is designed for nearly circular low earth orbit, and
%   ignores all forces except gravity from env_gravity.
%   Using higher order integrators from 
%   https://doi.org/10.1016/0375-9601(90)90092-3
%
%   r_ecef(matrix (3,1)):
%       position in ecef coordinate system (m)
%   v_ecef(matrix (3,1)):
%       velocity in ecef coordinate system (m/s)
%   dt(double):
%       Time step, how much time to update the orbit (s)
%   start_time(double): seconds since const.INITGPS_WN
%
% Started by Nathan Zimmerberg on Jan 30, 2020
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Jan 31, 2020
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
global const

%% ecef->ecef0
r_ecef0= r_ecef;
v_ecef0= v_ecef+cross(const.earth_rate_ecef,r_ecef);
%% get circular orbit reference
energy= 0.5*dot(v_ecef0,v_ecef0)-const.mu/norm(r_ecef0);
a=-const.mu/2/energy;
h_ecef0= cross(r_ecef0,v_ecef0);
x= r_ecef0;
x=x/norm(x)*a;
y= cross(h_ecef0,r_ecef0);
y=y/norm(y)*a;
omega= h_ecef0/norm(h_ecef0)*sqrt(const.mu/(a*a*a));
t0=0;
[orb_r,orb_v]= circular_orbit(0,t0,omega,x,y);
rel_r= r_ecef0-orb_r;
rel_v= v_ecef0-orb_v;
%% get higher order integrator steps this should be done at compile time in C++
%high order integrators
%https://doi.org/10.1016/0375-9601(90)90092-3
%https://pdf.sciencedirectassets.com/271541/1-s2.0-S0375960100X05075/1-s2.0-0375960190900923/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEHIaCXVzLWVhc3QtMSJGMEQCIF1ECb38EYCxayPQRI0hpkbfjJc8eQ%2FAlwjL8FEtSVN1AiAg%2BNndqK38eCFIGiNiIh1DajL%2FBlIVrQFa6idjCf4l9Cq9Awj7%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAIaDDA1OTAwMzU0Njg2NSIMsEfyy6bWwA0G0iHLKpEDA%2FBHuVZ%2BVrJnY6l8heiEZ7jxtjL2zdPELqgIjgiJUo50nM%2FMAOinJx82WVIbukNl56aGWSCe6Pysdn2mS9cQx%2BrF2HAoTs%2BpiBadw%2FuAtggU4RQLc9%2Ffu4by%2FaCZYOYkTV7dA8KYAibKr76LaKZmZcal%2B2iHVc6pZom5iof3O%2FIMV%2BYiKuxkGKv9aiztBwn59PsE0v4odCDkbfs%2FNeIzNNjvZOqMd84xXSKB%2FND4TJArkhpBBo%2F9DShQp4wBwZepjKHaDAlmSJbyXt1nvlkr2c55HSXYMFzA0HocbScbekInnxoVRRIBhSlAJqVKLeTKScO%2B3%2BEuIJM9i3Mdf%2FZF4LfNdQad1Edalc6Xg7seo1Cm13aX92SIbBql8JIHQrhL%2F8YO4CNgtv6f4ajHqprlsq4Dbd6hVeM%2F%2F9pO7cyBibdRZMFDFKj%2Bn1yNx6voOQna4fXd9953vNwVer4TbZEMuxAI3gyte7V9uxc4qqi33wYavVFpBeHv8%2FAp1u5pE0h4NvB7SCTJ2VFEl7m6mKN5mIQwsLSZ8QU67AFWLmQMZ2r2Bx6Eos9Sx7Q9MQju0X5Dacz5rMWai7MeWLv4cbY05t%2BFzppKCGLTEE%2B80u6E59onOWV1ppmvgYYD%2BH%2FxMskaYil9mZuhhfSkrVbTEbUmyWIURFu76ht72ZGzfLiuw35ilknpQoeewBNIGwvEn%2FWbHsv1vrmn0JZk2TNVZBDaS7jEXpJINs7%2BJKo%2F82ngZKaGLwMLH%2BDUrbdd%2B0WqjiKX6y%2BAbBVpLjlZsRj%2BPoV2d97UR6ABtYSVHv%2FWITdO7Wd930KQQizMrh3yZDDDUrj4UQAqWpqqZv38OQ2HvDHcvhvx4wowyA%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20200121T024645Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYSQKDTZ5H%2F20200121%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=087c847babff243b457807c797f220e3743de6d63902d0943a61cd30e30ee95b&hash=4a586ea181fec35778b5f7e60a625cdd055f00d9ea16572cef4a41192adaebc9&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=0375960190900923&tid=spdf-306995ff-861f-40bd-9c97-a080a5d8c1b2&sid=0f7242a41192874f6a6a8bb-6ac9f383dac9gxrqa&type=client      
w6a= [-0.117767998417887E1,
    0.235573213359357E0,
    0.784513610477560E0,];
w=w6a;
m=length(w);
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
%% Iterations
N= ceil(abs(duration)/15/(m*2+1));
dt= duration/(N);
for i=1:N
    t= (i-1)*dt;
    for j= 1:length(d)
        % drift positions dt*c(j)
        rel_r= rel_r+rel_v*dt*c(j);
        t= t+dt*c(j);
        % calc acceleration at the half step
        [orb_r,~]= circular_orbit(t,t0,omega,x,y);
        now = start_time + t; %current time in seconds
        dcm_ecef_ecef0 = relative_earth_dcm(t);
        pos_ecef= dcm_ecef_ecef0*(orb_r+rel_r);
        % perturbations due to J-coefficients; returns acceleration in ECEF
        g_ecef=env_gravity(now,pos_ecef);
        %convert to ECEF0
        acc= dcm_ecef_ecef0'*g_ecef+const.mu*orb_r/a^3;
        % kick velocity dt*d(j)
        rel_v= rel_v + acc*dt*d(j);
    end
    % final drift positions dt*c(end)
    rel_r= rel_r+rel_v*dt*c(end);
    % reset refernece orbit
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
%% ecef0->ecef
dcm_ecef_ecef0 = relative_earth_dcm(duration);
r_ecef= dcm_ecef_ecef0*r_ecef0;
v_ecef= dcm_ecef_ecef0*v_ecef0;
v_ecef= v_ecef-cross(const.earth_rate_ecef,r_ecef);
end

function [r,v]= circular_orbit(t,t0,omega,x,y)
    %Returns the position from circular orbit at time t in ecef0
    theta= (t-t0)*norm(omega);
    r= x*cos(theta)+y*sin(theta);
    v= cross(omega,r);
end

function [A_EI] = relative_earth_dcm(dt)
    %relative_earth_dcm returns the dcm to rotate from initial ecef to ecef
    %dt seconds latter.
    global const
    earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
    earth_omega= norm(const.earth_rate_ecef);
    theta= earth_omega*dt;% earth rotation angle
    A_EI= cos(theta)*eye(3)-sin(theta)*cross_product_matrix(earth_axis)+(1-cos(theta))*(earth_axis*earth_axis');
end

function M = cross_product_matrix(x)
    %cross_product_matrix 
    % see eq 2.55 in the ADC book
    M=[ 0 -x(3) x(2);
        x(3) 0 -x(1);
        -x(2) x(1) 0;];
end