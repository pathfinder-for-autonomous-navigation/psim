function [detumbler_state] = initialize_detumbler_state()
%initialize_detumbler_state Initializes the detumbler state
detumbler_state.counter=0;
detumbler_state.old_magnetic_field=zeros(3,1);
detumbler_state.magrod_moment_cmd= zeros(3,1);
end

