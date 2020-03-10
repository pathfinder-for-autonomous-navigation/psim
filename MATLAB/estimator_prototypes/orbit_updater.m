function state = orbit_updater(state,t0,dt)
%ORBIT_UPDATER updates an orbit where state is [r;v] in mks, ECEF dt is
%seconds
%uses 40x40 gravity model
perturbs.drag = 1;
perturbs.solrad = 0;
perturbs.bodmoon = 0;
perturbs.bodsun = 0;
perturbs.numJs = 10;
startr_ecef=state(1:3);
startv_ecef=state(4:6);
[r_final,v_final]  = true_orbit_propagator9(startr_ecef, startv_ecef, t0, dt, perturbs);
%[r_final2,v_final2]  = true_orbit_propagator2(startr_ecef, startv_ecef, t0, dt, perturbs);
state(1:3)=r_final;
state(4:6)=v_final;
end

