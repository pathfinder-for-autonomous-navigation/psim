function [computer_state_follower,computer_state_leader]= initialize_computer_states(condition);
%initialize_computer_states Constructs both leader and follower computer states.
computer_state_follower=struct();
computer_state_follower.adcs_state='init';

computer_state_leader=struct();
computer_state_leader.adcs_state='init';
end
