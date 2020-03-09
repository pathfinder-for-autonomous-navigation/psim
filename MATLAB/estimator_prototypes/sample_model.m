function [truth,measures] = sample_model(n,initializer,updater,measurer,dt,t0)
%sample_orbit Samples the orbit and outputs measures
%   truth is a (m,n) matrix of state
%   measures is a (p,n) matrix of measurements
%   n is the number of steps to go
%   initializer is a function with no inputs that outputs an initial the system
%       state
%   updater(state,t0,dt) is a function that takes in the state as a column vec at time t0
%       and outputs the new state at time t0+dt
%   measurer(state,t) is a funtion that makes a mesurement of the state at
%       time t
%   dt is the time step
%   t0 is the start time
truth= nans(6,n);
measures= nans(6,n);
truth(:,1)=feval(initializer);
measures(:,1)= feval(measurer,truth(:,1),t0);
t=t0;
for i=2:n
    truth(:,i)=feval(updater,truth(:,i-1),t,dt);
    t=t+dt;
    measures(:,i)= feval(measurer,truth(:,i),t);
end
end

