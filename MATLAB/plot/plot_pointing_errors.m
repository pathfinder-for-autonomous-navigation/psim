function plot_pointing_errors(main_state_trajectory)
%plot_pointing_errors plots magnitude of error of where the sats are pointing vs where they should be pointing.
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
docking_face_angle=zeros(1, N);
antenna_angle_f=zeros(1, N);
antenna_angle_l=zeros(1, N);
%run through the trajectory and store values
for n=1:N
    dynamics_f= main_state_trajectory{n}.follower.dynamics;
    dynamics_l= main_state_trajectory{n}.leader.dynamics;
    pos_eci= get_truth('position eci',dynamics_f);
    r_hat= pos_eci/norm(pos_eci);
    antenna_angle_f(n)=acos(dot(r_hat,get_truth('antenna eci',dynamics_f)))*180/pi;
    pos_eci= get_truth('position eci',dynamics_l);
    r_hat= pos_eci/norm(pos_eci);
    antenna_angle_l(n)=acos(dot(r_hat,get_truth('antenna eci',dynamics_l)))*180/pi;
    docking_face_angle(n)=acos(-dot(get_truth('docking face eci',dynamics_f),get_truth('docking face eci',dynamics_l)))*180/pi;
    t(n) = dynamics_f.time;  % Take initial data
end


figure;
plot(t,[antenna_angle_f; antenna_angle_l;docking_face_angle ])
legend('follower Quake antenna pointing error','leader Quake antenna pointing error','docking face pointing error')
title('Pointing errors')
xlabel('time (s)')
ylabel('angle error (degrees)')
