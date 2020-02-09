function [h1,h2] = average_angular_momentum(r1, v1, r2, v2)
% Returns the averaged angular momentum in the inertial frame
h1= cross(r1,v1);
h2= cross(r2,v2);
return
opt = odeset('RelTol', 1e-5, 'AbsTol', 1e-3);
t_orbit = 1.5*60*60;
samples= 10;
times=linspace(0,t_orbit,samples);
[t, y] = ode113(@frhs, times, [r1; v1; r2; v2], opt);
t_ind= zeros(samples,1);
for i=1:samples
    t_ind(i)=find(t==times(i));
end
r1 = y(t_ind, 1:3)';
v1 = y(t_ind, 4:6)';
r2 = y(t_ind, 7:9)';
v2 = y(t_ind, 10:12)';
h1= cross(r1,v1);
h2= cross(r2,v2);
h1= sum(h1,2)/samples;
h2= sum(h2,2)/samples;

end

function dy = frhs(~, y)

dy = zeros(12, 1);

[acceleration,~,~]= env_gravity(0,y(1:3));
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = acceleration;

[acceleration,~,~]= env_gravity(0,y(7:9));
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = acceleration;

end