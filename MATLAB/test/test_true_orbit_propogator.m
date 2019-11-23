config();

%load GRACE data; in m, m/s, ECEF
data = csvread('graceClean.csv',1,0);

%duration of the mission you want to simulate
num_states = 5400; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day

global const

%convert to from ECEF --> ECI
for i = 1:num_states
   %GRACE time is gps_time in continuous seconds past 01-Jan-2000 11:59:47 UTC
   
   %convert GRACE time to PAN time
   pantime = utl_grace2pantime(data(i,1));
   [r_ECI, v_ECI] = ECEFtoECI(pantime,data(i,2:4)',data(i,8:10)');

   %[r_ECI, v_ECI] = ECEFtoECI(data(i,1),data(i,2:4)',data(i,8:10)');
   
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%test cases: [drag solrad 3rdbodymoon 3rdbodysun numberofJ's]
% run true orbit propogator with GRACE initial data
% [t_array, states, orb_elemsf] = true_orbit_propagator(r,v,start_time,duration, perturbs)
% start time in GPS time in seconds, duration of mission to look at in seconds


%simplest model, J2 only
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 2;

[t_array1, states1, orb_elemsf1] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err1 = get_error(data, states1);
saveas(gcf,'test1.png')
time1 = toc;

%J4
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 4;

[t_array2, states2, orb_elemsf2] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err2 = get_error(data, states2);
saveas(gcf,'test2.png')
time2 = toc;
 
%J6
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 6;

[t_array3, states3, orb_elemsf3] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err3 = get_error(data, states3);
saveas(gcf,'test3.png')
time3 = toc;

%J8
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 8;

[t_array4, states4, orb_elemsf4] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err4 = get_error(data, states4);
saveas(gcf,'test4.png')
time4 = toc;

%J10
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[t_array5, states5, orb_elemsf5] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err5 = get_error(data, states5);
saveas(gcf,'test5.png')
time5 = toc;

%test drag
tic
perturbs.drag = 1;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[t_array6, states6, orb_elemsf6] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err6 = get_error(data, states6);
saveas(gcf,'test6.png')
time6 = toc;


%test solrad
tic
perturbs.drag = 0;
perturbs.solrad = 1;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[t_array7, states7, orb_elemsf7] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err7 = get_error(data, states7);
saveas(gcf,'test7.png')
time7 = toc;

%test 3rd body moon
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 1;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[t_array8, states8, orb_elemsf8] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err8 = get_error(data, states8);
saveas(gcf,'test8.png')
time8 = toc;

%test 3rd body sun
tic
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 1;
perturbs.numJs = 10;

[t_array9, states9, orb_elemsf9] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err9 = get_error(data, states9);
saveas(gcf,'test9.png')
time9 = toc;

%high fidelity
tic
perturbs.drag = 1;
perturbs.solrad = 1;
perturbs.bodmoon = 1;
perturbs.bodsun = 1;
perturbs.numJs = 10;

[t_array10, states10, orb_elemsf10] = true_orbit_propagator(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);
err10 = get_error(data, states10);
saveas(gcf,'test10.png')
time10 = toc;


%%%%compare true orbital states with GRACE states
function err = get_error(data, states)
    err = zeros(length(states),6);
    %[xerr, yerr, zerr, xdoterr, ydoterr, zdoterr]
    
    for i = 1:length(states)        

        err(i,1) = abs(data(i,2)-states(i,1)); %diff in position in m
        err(i,2) = abs(data(i,3)-states(i,2));
        err(i,3) = abs(data(i,4)-states(i,3));
        err(i,4) = abs(data(i,8)-states(i,4)); %diff in velocity in m/s
        err(i,5) = abs(data(i,9)-states(i,5));
        err(i,6) = abs(data(i,10)-states(i,6));
  
    end
end

function getPlots(t_array,orb_elemsf) 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PLOTS for true orbit propogator
    figure(2)
    
    subplot(5,2,1)
    loglog(t_array,orb_elemsf(:,1),'LineWidth',1);                     
    ylabel('a'); 
    xlabel('time (s)'); 
    title('a semi major axis');
    
    subplot(5,2,2)
    loglog(t_array,orb_elemsf(:,2),'LineWidth',1);                    
    ylabel('eMag'); 
    xlabel('time (s)');
    title('eccentricity');
    
    subplot(5,2,3)
    loglog(t_array,orb_elemsf(:,3),'LineWidth',1);                   
    ylabel('i radians'); 
    xlabel('time (s)');
    title('i inclination radians');
    
    subplot(5,2,4)
    loglog(t_array,orb_elemsf(:,4),'LineWidth',1);                     
    ylabel('O radians'); 
    xlabel('time (s)');
    title('O RAAN radians'); 
    
    subplot(5,2,5)
    loglog(t_array,orb_elemsf(:,5),'LineWidth',1);                     
    ylabel('o radians'); 
    xlabel('time (s)');
    title('o argument of perigee radians');
    
    subplot(5,2,6)
    loglog(t_array,orb_elemsf(:,6),'LineWidth',1);                     
    ylabel('M radians'); 
    xlabel('time (s)');
    title('M mean anomaly radians');
    
    subplot(5,2,7)
    loglog(t_array,orb_elemsf(:,7),'LineWidth',1);                   
    ylabel('trueLon radians'); 
    xlabel('time (s)');
    title('trueLon true longitude radians'); 
    
    subplot(5,2,8)
    loglog(t_array,orb_elemsf(:,8),'LineWidth',1);                
    ylabel('argLat radians'); 
    xlabel('time (s)');
    title('argLat radians');
    
    subplot(5,2,9)
    loglog(t_array,orb_elemsf(:,9),'LineWidth',1);                      
    ylabel('lonPer radians'); 
    xlabel('time (s)');
    title('lonPer radians'); 
    
    subplot(5,2,10)
    loglog(t_array,orb_elemsf(:,10),'LineWidth',1);                     
    ylabel('p'); 
    xlabel('time (s)');
    title('p semilatus rectum');

end