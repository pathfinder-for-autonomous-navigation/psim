config();

%load GRACE data; in m, m/s, ECEF
data = csvread('graceClean.csv',1,0);

%duration of the mission you want to simulate
num_states = 23*60*60; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day

global const

starttime = utl_grace2pantime(data(1,1));
startr_ecef= data(1,2:4)';
startv_ecef= data(1,8:10)';
stoptime= utl_grace2pantime(data(1,1));
stopr_ecef= data(1+num_states,2:4)';
stopv_ecef= data(1+num_states,8:10)';
earth_axis= const.earth_rate_ecef/norm(const.earth_rate_ecef);
theta= norm(const.earth_rate_ecef)*num_states;% earth rotation angle
quat_ecef_ecef0p= [earth_axis*sin(theta/2);cos(theta/2);];

fprintf('converted GRACE data from ECEF --> ECI \n')

%getting orbital elements for GRACE
orb_elemsg = zeros(length(data),10); 
t_array = linspace(data(1,1),data(1,1)+length(data),1);
for count = 1:length(data)
    rf  = data(count,2:4)'; %new position 
    vf = data(count,8:10)'; %new velocity 
    %quatf = states(count,7:10); %new quaternions

    %calculate orbital elements
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
perturbs.bodmoon = 1;
perturbs.bodsun = 1;
perturbs.numJs = 10;

[r_final,v_final]  = true_orbit_propagator3(data(1,2:4), data(1,8:10), data(1,1), num_states, perturbs);


error_r= stopr_ecef-r_final
%error_v= stopv_ecef-v_final
time1 = toc;

