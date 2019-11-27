function [computer_state_follower,computer_state_leader]= initialize_computer_states(condition);
%initialize_computer_states Constructs both leader and follower computer states.
computer_state_follower=struct();
computer_state_follower.main_state='off';
computer_state_follower.type='follower';
computer_state_follower.initialization_hold_done=true;

computer_state_leader=struct();
computer_state_leader.main_state='off';
computer_state_leader.type='leader';
computer_state_leader.initialization_hold_done=true;


end

