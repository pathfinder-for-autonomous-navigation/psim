function [computer_state_follower,computer_state_leader]= initialize_computer_states(condition);
%initialize_computer_states Constructs both leader and follower computer states.
computer_state_follower=struct();
computer_state_follower.main_state='off';
computer_state_follower.type='follower';
computer_state_follower.on_time=0;
computer_state_follower.mag_bias_est_state= adcs_initialize_mag_bias_est();
computer_state_follower.detumbler_state=adcs_initialize_detumbler_state();
computer_state_follower.num_magnetometer_bias_readings=zeros(3,1);

computer_state_leader=struct();
computer_state_leader.main_state='off';
computer_state_leader.type='leader';
computer_state_leader.on_time=0;
computer_state_leader.mag_bias_est_state= adcs_initialize_mag_bias_est();
computer_state_leader.detumbler_state=adcs_initialize_detumbler_state();
computer_state_leader.num_magnetometer_bias_readings=zeros(3,1);


end

