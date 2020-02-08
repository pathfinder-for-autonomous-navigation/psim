config();

%load GRACE data; in m, m/s, ECEF
data = csvread('graceClean.csv',1,0);

%duration of the mission you want to simulate
num_states = 1*60*60; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day

global const
global count
count=0;

starttime = utl_grace2pantime(data(1,1));
startr_ecef= data(1,2:4)';
startv_ecef= data(1,8:10)';
stopr_ecef= data(1+num_states,2:4)';
stopv_ecef= data(1+num_states,8:10)';



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
perturbs.numJs = 10;

[r_final,v_final]  = true_orbit_propagator9(startr_ecef, startv_ecef, starttime, num_states, perturbs);
[r_final2,v_final2]  = true_orbit_propagator2(startr_ecef, startv_ecef, starttime, num_states, perturbs);

error_r= stopr_ecef-r_final
error_v= stopv_ecef-v_final
r_final2-r_final
v_final2-v_final
count


