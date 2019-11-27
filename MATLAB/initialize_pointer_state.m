function [pointer_state] = initialize_pointer_state()
%initialize_pointer_state Initializes the pointer state
global const
pointer_state=struct();
%Initialize moving median filter
pointer_state.derivative_buffer=zeros(3,const.ATTITUDE_PD_derivative_buffer_size);
pointer_state.derivative_buffer_location=0;
pointer_state.last_error= zeros(3,1);
end

