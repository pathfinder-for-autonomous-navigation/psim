%load GRACE data; in m, m/s, ECEF
rng(2,'threefry');
data = csvread('graceClean.csv',1,0);
N=10000;
dt= 0.2;

%duration of the mission you want to simulate
num_states = N*dt; %1.5 hours i.e approximately 1 orbit %length(data) for 1 day

global const

starttime = utl_grace2pantime(data(1,1));
startr_ecef= data(1,2:4)';
startv_ecef= data(1,8:10)';
stoptime= utl_grace2pantime(data(1,1))+num_states;
stopr_ecef= data(1+num_states,2:4)';
stopv_ecef= data(1+num_states,8:10)';

perturbs.drag = 0;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;

[r_final,v_final]  = true_orbit_propagator2(startr_ecef, startv_ecef, starttime, num_states, perturbs);

% test just the self propagator
r_ecef= startr_ecef;
v_ecef= startv_ecef;
rel_target_r_ecef= nan(3,1);
rel_target_v_ecef= nan(3,1);
for i= 1:(N)
    [r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] ... 
        = orb_short_orbit_prop(...
        r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,starttime+dt*i);
end
assert(norm(r_final-r_ecef)<0.01,"position too far off from reference integrator "+norm(r_final-r_ecef))
assert(norm(v_final-v_ecef)<1E-4,"velocity too far off from reference integrator "+norm(v_final-v_ecef))

% test the target propagator
r_ecef= startr_ecef-1E4*ones(3,1);
v_ecef= startv_ecef-1.0*ones(3,1);
rel_target_r_ecef= 1E4*ones(3,1);
rel_target_v_ecef= 1*ones(3,1);
for i= 1:(N)
    [r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] ... 
        = orb_short_orbit_prop(...
        r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,starttime+dt*i);
end
assert(norm(r_final-r_ecef-rel_target_r_ecef)<1E-2,"position too far off from reference integrator "+norm(r_final-r_ecef-rel_target_r_ecef))
assert(norm(v_final-v_ecef-rel_target_v_ecef)<1E-4,"velocity too far off from reference integrator "+norm(v_final-v_ecef-rel_target_v_ecef))

% test reversibility of propagator
r_ecef= r_final-1E4*ones(3,1);
v_ecef= v_final-1.0*ones(3,1);
rel_target_r_ecef= 1E4*ones(3,1);
rel_target_v_ecef= 1*ones(3,1);
for i= 1:(N)
    [r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] ... 
        = orb_short_orbit_prop(...
        r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,-dt,starttime+dt*(N+1)-dt*i);
end
assert(norm(startr_ecef-r_ecef-rel_target_r_ecef)<1E-2,"position too far off from reference integrator "+norm(startr_ecef-r_ecef-rel_target_r_ecef))
assert(norm(startv_ecef-v_ecef-rel_target_v_ecef)<1E-4,"velocity too far off from reference integrator "+norm(startv_ecef-v_ecef-rel_target_v_ecef))

% test jacobians with finite difference
for i=1:100
    r_ecef= startr_ecef;
    v_ecef= startv_ecef;
    start_r_rel= 1E-1*randn(3,1);
    start_v_rel= 1E-1*randn(3,1);
    rel_target_r_ecef= start_r_rel;
    rel_target_v_ecef= start_v_rel;
    J=eye(6);
    [r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] ... 
        = orb_short_orbit_prop(...
        r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,starttime);
    J=jacobian*J;
    error= norm(J*[start_r_rel; start_v_rel;]-[rel_target_r_ecef; rel_target_v_ecef;]);
    assert(error<1E-9,"self jacobian finite diff failed "+ error)
end

% test target jacobians with finite difference
r_ecef= startr_ecef;
v_ecef= startv_ecef;
r0= 1E5*ones(3,1);
v0= 1E2*ones(3,1);
[~,~,~,r1,v1,J] ... 
    = orb_short_orbit_prop(...
    r_ecef,v_ecef,r0,v0,dt,starttime);
for i=1:100
    start_r_rel= 1E-1*randn(3,1);
    start_v_rel= 1E-1*randn(3,1);
    rel_target_r_ecef= r0+start_r_rel;
    rel_target_v_ecef= v0+start_v_rel;
    [~,~,~,rel_target_r_ecef,rel_target_v_ecef,~] ... 
        = orb_short_orbit_prop(...
        r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,starttime);
    error= norm(J*[start_r_rel; start_v_rel;]-[rel_target_r_ecef-r1; rel_target_v_ecef-v1;]);
    assert(error<1E-9,"self jacobian finite diff failed "+ error)
end

% test that the jacobian is symplectic
r_ecef= startr_ecef;
v_ecef= startv_ecef;
rel_target_r_ecef= 1E5*ones(3,1);
rel_target_v_ecef= 1E2*ones(3,1);
[r_ecef,v_ecef,jacobian,rel_target_r_ecef,rel_target_v_ecef,target_jacobian] ... 
	= orb_short_orbit_prop(...
	r_ecef,v_ecef,rel_target_r_ecef,rel_target_v_ecef,dt,starttime);
assert(abs(det(jacobian)-1)<1E-12,"Jacobian isn't symplectic")
assert(abs(det(target_jacobian)-1)<1E-12,"Jacobian isn't symplectic")