function plot_pointing_errors(main_state_trajectory)
%plot_pointing_errors plots magnitude of error of where the sats are pointing vs where they should be pointing.
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
%these angles are in degrees
docking_face_angle_f=zeros(1, N);
docking_face_angle_l=zeros(1, N);
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
    
    relative_pos_eci=get_truth('position eci',dynamics_l)-get_truth('position eci',dynamics_f);
    relative_pos_eci_hat=relative_pos_eci/norm(relative_pos_eci);
    
    docking_face_angle_l(n)=acos(-dot(get_truth('docking face eci',dynamics_l),relative_pos_eci_hat))*180/pi;
    docking_face_angle_f(n)=acos(dot(get_truth('docking face eci',dynamics_f),relative_pos_eci_hat))*180/pi;

    t(n) = dynamics_f.time;  % Take initial data
end


figure;
plot(t,[antenna_angle_f; antenna_angle_l;docking_face_angle_f;docking_face_angle_l ])
legend('follower Quake antenna pointing error','leader Quake antenna pointing error','follower docking face pointing error','leader docking face pointing error')
title('Pointing errors')
xlabel('time (s)')
ylabel('angle error (degrees)')
