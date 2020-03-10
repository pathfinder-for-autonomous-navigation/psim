%function test_orbit_estimator()
global dt
dt=0.1;
SQ= diag([zeros(3,1);1E-8*ones(3,1)]);
messdiv= 10*ones(6,1);
[estimator,initest] = sqrt_EKF_addnoise(@jac_updaterfn,SQ,messdiv);
[estimates,states] = sample_estimation(estimator,initest,measures);
figure;
plot(vecnorm(estimates(1:3,:)-truth(1:3,:)))
figure;
plot(vecnorm(estimates(4:6,:)-truth(4:6,:)))

function [J,x]=jac_updaterfn(x)
    global dt
    [x(1:3),x(4:6),J,~,~,~] = orb_short_orbit_prop(x(1:3),x(4:6),zeros(3,1),zeros(3,1),dt,0);
end