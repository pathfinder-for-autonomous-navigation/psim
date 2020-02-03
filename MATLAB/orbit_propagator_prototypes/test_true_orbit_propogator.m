config();

%load GRACE data; in m, m/s, ECEF
data = csvread('graceClean.csv',1,0);

%duration of the mission you want to simulate
num_states = 13*60*60; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day

global const

starttime = utl_grace2pantime(data(1,1));
startr_ecef= data(1,2:4)';
startv_ecef= data(1,8:10)';
stoptime= starttime+num_states;
stopr_ecef= data(1+num_states,2:4)';
stopv_ecef= data(1+num_states,8:10)';
[startr_eci, startv_eci] = ECEFtoECI(starttime,startr_ecef,startv_ecef);
[stopr_eci, stopv_eci] = ECEFtoECI(stoptime,stopr_ecef,stopv_ecef);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%test cases: [drag solrad 3rdbodymoon 3rdbodysun numberofJ's]
% run true orbit propogator with GRACE initial data
% [t_array, states, orb_elemsf] = true_orbit_propagator(r,v,start_time,duration, perturbs)
% start time in GPS time in seconds, duration of mission to look at in seconds


%simplest model, J2 only
perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[t_array1, states1, orb_elemsf1] = true_orbit_propagator(startr_eci, startv_eci, starttime, num_states, perturbs);
[r_final,v_final]  = true_orbit_propagator3(startr_ecef, startv_ecef, starttime, num_states, perturbs);
[finalr_eci, finalv_eci] = ECEFtoECI(stoptime,r_final,v_final);
% err1 = get_error(data, states1);
error_r= stopr_eci-states1(end,1:3)'
states1(end,1:3)'-finalr_eci

