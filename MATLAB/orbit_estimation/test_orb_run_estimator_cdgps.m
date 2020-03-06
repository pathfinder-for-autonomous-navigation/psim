%load GRACE data; in m, m/s, ECEF
config();
rng(2,'threefry');
data = csvread('graceClean.csv',1,0);
state = orb_initialize_estimator();
dt= int64(121E6);
%duration of the mission you want to simulate
num_states = 10*60*60; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day
N= floor(num_states*1E9/dt);
global const

starttime = utl_grace2pantime(data(1,1));
startr_ecef= data(1,2:4)';
startv_ecef= data(1,8:10)';
start_target_time = utl_grace2pantime(data(1,1));
start_target_r_ecef= data(1+1,2:4)';
start_target_v_ecef= data(1+1,8:10)';
[state,self2target_r_ecef,self2target_v_ecef,r_ecef,v_ecef] ...
    = orb_run_estimator(...
    state, ...
    true, ...
    false, ...
    false, ...
    startr_ecef+0.0*randn(3,1), ...
    startv_ecef+0.0*randn(3,1), ...
    nan(3,1), ...
    int64(starttime)*int64(1E9), ...
    start_target_r_ecef+0.0*randn(3,1), ...
    start_target_v_ecef+0.0*randn(3,1), ...
    int64(start_target_time)*int64(1E9));
sat_time=int64(0);
seconds= 1;
state_trajectory = cell(1,N);
j=1;
clear errors_pos
clear errors_vel
clear errors_rel_pos
clear errors_rel_vel
for i=1:N
    "NEW CYCLE /n /n /n";
    sat_time= sat_time+dt;
    if(sat_time>= int64(1E9))
        sat_time = sat_time-int64(1E9);
        seconds= seconds+1
        gps_time = int64(utl_grace2pantime(data(seconds,1)))*int64(1E9);
        true_r_ecef= data(seconds,2:4)';
        true_v_ecef= data(seconds,8:10)';
        gps_r_ecef= data(seconds,2:4)'+1.0*randn(3,1);
        gps_v_ecef= data(seconds,8:10)'+1.0*randn(3,1);
        true_target_r_ecef= data(seconds+1,2:4)';
        true_target_v_ecef= data(seconds+1,8:10)';
        gps_self2target_r_ecef= true_target_r_ecef-true_r_ecef+0.01*randn(3,1);
    else
        gps_time = int64(seconds)*int64(1E9)+sat_time+int64(starttime)*int64(1E9);
        true_r_ecef= nan(3,1);
        true_v_ecef= nan(3,1);
        true_target_r_ecef= nan(3,1);
        true_target_v_ecef= nan(3,1);
        gps_r_ecef= nan(3,1);
        gps_v_ecef= nan(3,1);
        gps_self2target_r_ecef= nan(3,1);
    end
    if(mod(seconds,90*60)> 90*60/2)
        %in eclipse, so no gps
        gps_r_ecef= nan(3,1);
        gps_v_ecef= nan(3,1);
        gps_self2target_r_ecef= nan(3,1);
    end
    [state,self2target_r_ecef,self2target_v_ecef,r_ecef,v_ecef] ...
    = orb_run_estimator(...
    state, ...
    true, ...
    false, ...
    false, ...
    gps_r_ecef, ...
    gps_v_ecef, ...
    gps_self2target_r_ecef, ...
    gps_time, ...
    nan(3,1), ...
    nan(3,1), ...
    int64(0));
    %state_trajectory{i}=state;
    if all(isfinite([true_r_ecef;true_v_ecef;true_target_r_ecef;true_target_v_ecef;]))
        errors_pos(:,j)= r_ecef-true_r_ecef;
        errors_vel(:,j)= v_ecef-true_v_ecef;
        errors_rel_pos(:,j)= true_target_r_ecef-true_r_ecef-self2target_r_ecef;
        errors_rel_vel(:,j)= true_target_v_ecef-true_v_ecef-self2target_v_ecef;
        j=j+1;
    end
end
figure;
plot(vecnorm(errors_pos))
title('error self position ecef')
xlabel('time (s)')
ylabel('Distance (m)')

figure;
plot(vecnorm(errors_vel))
title('error self velocity ecef')
xlabel('time (s)')
ylabel('Speed (m/s)')

figure;
plot(vecnorm(errors_rel_pos))
title('error target relative position ecef')
xlabel('time (s)')
ylabel('Distance (m)')

figure;
plot(vecnorm(errors_rel_vel))
title('error target relative velocity ecef')
xlabel('time (s)')
ylabel('Speed (m/s)')
