function plot_get_truth(main_state_trajectory,name,sat_name)
%plot_get_truth makes plots of the get_truth value over time
%   Create a plot over time of `name` truth value from `sat_name`.
%         `name` must be an acceptable input to `get_truth()`,
%         `sat_name` should be either `'follower'` or `'leader'`
N = length(main_state_trajectory);  % Number of samples
t = zeros(1, N);               % Stored mission time data
values=[];
%run through the trajectory and store values
for n=1:N
    if startsWith(sat_name,"f",'IgnoreCase',true)
        dynamics= main_state_trajectory{n}.follower.dynamics;
    else 
        dynamics= main_state_trajectory{n}.leader.dynamics;
    end
    values= [values get_truth(name,dynamics)];
    t(n) = dynamics.time;  % Take initial data
end
%check if value is a scalor or vector
s=size(values);
if s(1)==1 
    figure;
    plot(t,values)
    title(name)
    xlabel('time (s)')
    ylabel(name)
end
if s(1)==3 
    figure;
    plot(t,[values; vecnorm(values)])
    legend('x','y','z','norm')
    title(name)
    xlabel('time (s)')
    ylabel(name)
end
