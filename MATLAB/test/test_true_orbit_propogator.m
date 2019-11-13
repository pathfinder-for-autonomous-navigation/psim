config();
tic
%load GRACE data; in m, m/s, ECEF
data = csvread('graceClean.csv',1,0);
%num_states = length(data);
num_states = 3;
global const

%convert to from ECEF --> ECI
for i = 1:num_states
   %convert time to Julian Date Vector
   datetime = utl_time2datetimeUTC(data(i,1),0); %second input should be init_GPS_week_number(int): initial GPS week number.  
   jd = juliandate(datetime); %default timezone is UTC
   
   [r_ECI, v_ECI] = ECEFtoECI(jd,data(i,2:4)',data(i,8:10)');
   data(i,2:4) = r_ECI; %convert position in ECEF to ECI
   data(i,8:10) = v_ECI; %convert velocity in ECEF to ECI
end

fprintf('converted GRACE data from ECEF --> ECI \n')

%getting orbital elements for GRACE
orb_elemsg = zeros(num_states,10); 
t_array = linspace(data(1,1),data(1,1)+num_states,1);
for count = 1:num_states
    rf  = data(count,2:4)'; %new position 
    vf = data(count,8:10)'; %new velocity 
    %quatf = states(count,7:10); %new quaternions

    %calculate orbital elements
    [a, eMag, i, O, o, nu, truLon, argLat, lonPer, p] = utl_rv2orb(rf, vf, const.mu);
    orb_elemsg(count,1) = a*1E3; orb_elemsg(count,2) = eMag; orb_elemsg(count,3) = i;
    orb_elemsg(count,4) = O; orb_elemsg(count,5) = o; orb_elemsg(count,6) = nu;
    orb_elemsg(count,7) = truLon; orb_elemsg(count,8) = argLat; orb_elemsg(count,9) = lonPer;
    orb_elemsg(count,10) = p*1E3;
end


% run true orbit propogator with GRACE initial data
% true_orbit_propagator(initial pos, initial vel, start_time in sec, duration in sec);
[t_array, states, orb_elemsf] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), length(data));
%[t_array, states, orb_elemsf] = true_orbit_propagator(data(1,1:3), data(1,7:9), 0, 1.314E7);

%%%%compare true orbital states with GRACE states
err = zeros(num_states,6);

for i = 1:num_states
    x_grace = data(i,2);
    y_grace = data(i,3);
    z_grace = data(i,4);
    xdot_grace = data(i,8);
    ydot_grace = data(i,9);
    zdot_grace = data(i,10);
    
    x = states(i,1);
    y = states(i,2);
    z = states(i,3);
    xdot = states(i,4);
    ydot = states(i,5);
    zdot = states(i,6);
    
    err(i,1) = abs(x_grace-x)/x_grace;
    err(i,2) = abs(y_grace-y)/y_grace;
    err(i,3) = abs(z_grace-z)/z_grace;
    err(i,4) = abs(xdot_grace-xdot)/xdot_grace;
    err(i,5) = abs(ydot_grace-ydot)/ydot_grace;
    err(i,6) = abs(zdot_grace-zdot)/zdot_grace;
end

avg_xerr = mean(err(:,1));
avg_yerr = mean(err(:,2));
avg_zerr = mean(err(:,3));
avg_xdoterr = mean(err(:,4));
avg_ydoterr = mean(err(:,5));
avg_zdoterr = mean(err(:,6));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PLOTS for true orbit propogator
% figure(1)
% 
% subplot(5,2,1)
% loglog(t_array,orb_elemsf(:,1),'LineWidth',1);                     
% ylabel('a'); 
% xlabel('time (s)'); 
% title('a semi major axis');
% 
% subplot(5,2,2)
% loglog(t_array,orb_elemsf(:,2),'LineWidth',1);                    
% ylabel('eMag'); 
% xlabel('time (s)');
% title('eccentricity');
% 
% subplot(5,2,3)
% loglog(t_array,orb_elemsf(:,3),'LineWidth',1);                   
% ylabel('i radians'); 
% xlabel('time (s)');
% title('i inclination radians');
% 
% subplot(5,2,4)
% loglog(t_array,orb_elemsf(:,4),'LineWidth',1);                     
% ylabel('O radians'); 
% xlabel('time (s)');
% title('O RAAN radians'); 
% 
% subplot(5,2,5)
% loglog(t_array,orb_elemsf(:,5),'LineWidth',1);                     
% ylabel('o radians'); 
% xlabel('time (s)');
% title('o argument of perigee radians');
% 
% subplot(5,2,6)
% loglog(t_array,orb_elemsf(:,6),'LineWidth',1);                     
% ylabel('M radians'); 
% xlabel('time (s)');
% title('M mean anomaly radians');
% 
% subplot(5,2,7)
% loglog(t_array,orb_elemsf(:,7),'LineWidth',1);                   
% ylabel('trueLon radians'); 
% xlabel('time (s)');
% title('trueLon true longitude radians'); 
% 
% subplot(5,2,8)
% loglog(t_array,orb_elemsf(:,8),'LineWidth',1);                
% ylabel('argLat radians'); 
% xlabel('time (s)');
% title('argLat radians');
% 
% subplot(5,2,9)
% loglog(t_array,orb_elemsf(:,9),'LineWidth',1);                      
% ylabel('lonPer radians'); 
% xlabel('time (s)');
% title('lonPer radians'); 
% 
% subplot(5,2,10)
% loglog(t_array,orb_elemsf(:,10),'LineWidth',1);                     
% ylabel('p'); 
% xlabel('time (s)');
% title('p semilatus rectum');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PLOTS for GRACE
% figure(2);
% 
% subplot(5,2,1)
% loglog(t_array,orb_elemsg(:,1),'LineWidth',1);                     
% ylabel('a'); 
% xlabel('time (s)'); 
% title('a semi major axis');
% 
% subplot(5,2,2)
% loglog(t_array,orb_elemsg(:,2),'LineWidth',1);                     
% ylabel('eMag'); 
% xlabel('time (s)');
% title('eccentricity');
% 
% subplot(5,2,3)
% loglog(t_array,orb_elemsg(:,3),'LineWidth',1);                      
% ylabel('i radians'); 
% xlabel('time (s)');
% title('i inclination radians');
% 
% subplot(5,2,4)
% loglog(t_array,orb_elemsg(:,4),'LineWidth',1);                    
% ylabel('O radians'); 
% xlabel('time (s)');
% title('O RAAN radians'); 
% 
% subplot(5,2,5)
% loglog(t_array,orb_elemsg(:,5),'LineWidth',1);                   
% ylabel('o radians'); 
% xlabel('time (s)');
% title('o argument of perigee radians');
% 
% subplot(5,2,6)
% loglog(t_array,orb_elemsg(:,6),'LineWidth',1);                    
% ylabel('M radians'); 
% xlabel('time (s)');
% title('M mean anomaly radians');
% 
% subplot(5,2,7)
% loglog(t_array,orb_elemsg(:,7),'LineWidth',1);                     
% ylabel('trueLon radians'); 
% xlabel('time (s)');
% title('trueLon true longitude radians'); 
% 
% subplot(5,2,8)
% loglog(t_array,orb_elemsg(:,8),'LineWidth',1);                   
% ylabel('argLat radians'); 
% xlabel('time (s)');
% title('argLat radians');
% 
% subplot(5,2,9)
% loglog(t_array,orb_elemsg(:,9),'LineWidth',1);                      
% ylabel('lonPer radians'); 
% xlabel('time (s)');
% title('lonPer radians'); 
% 
% subplot(5,2,10)
% loglog(t_array,orb_elemsg(:,10),'LineWidth',1);                     
% ylabel('p'); 
% xlabel('time (s)');
% title('p semilatus rectum');
%  
toc