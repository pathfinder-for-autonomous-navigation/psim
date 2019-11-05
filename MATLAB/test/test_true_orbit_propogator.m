config();

%[t_array, states, orb_elemsf] = true_orbit_propagator([(500E3+6.37814E06) (500E3+6.37814E06) (500E3+6.37814E06)], [5 6 7], 0, 1.314E7);
[t_array, states, orb_elemsf] = true_orbit_propagator(1E3.*[2067829.846415895 776905.9724453629 0.0009578283838282736], [0.0006236271904723294 0.0006257082914522712 941.0211143841228], 0, 1.314E7);

%PLOTS
figure(1);

subplot(5,2,1)
plot(t_array,orb_elemsf(:,1),'LineWidth',1); 
axis equal; shg                     
ylabel('a km'); 
xlabel('time (s)'); 
title('a semi major axis km');

subplot(5,2,2)
plot(t_array,orb_elemsf(:,2),'LineWidth',1); 
axis equal; shg                     
ylabel('eMag'); 
xlabel('time (s)');
title('eccentricity');

subplot(5,2,3)
plot(t_array,orb_elemsf(:,3),'LineWidth',1); 
axis equal; shg                     
ylabel('i radians'); 
xlabel('time (s)');
title('i inclination radians');

subplot(5,2,4)
plot(t_array,orb_elemsf(:,4),'LineWidth',1); 
axis equal; shg                     
ylabel('O radians'); 
xlabel('time (s)');
title('O RAAN radians'); 

subplot(5,2,5)
plot(t_array,orb_elemsf(:,5),'LineWidth',1); 
axis equal; shg                     
ylabel('o radians'); 
xlabel('time (s)');
title('o argument of perigee radians');

subplot(5,2,6)
plot(t_array,orb_elemsf(:,6),'LineWidth',1); 
axis equal; shg                     
ylabel('M radians'); 
xlabel('time (s)');
title('M mean anomaly radians');

subplot(5,2,7)
plot(t_array,orb_elemsf(:,7),'LineWidth',1); 
axis equal; shg                     
ylabel('trueLon radians'); 
xlabel('time (s)');
title('trueLon true longitude radians'); 

subplot(5,2,8)
plot(t_array,orb_elemsf(:,8),'LineWidth',1); 
axis equal; shg                     
ylabel('argLat radians'); 
xlabel('time (s)');
title('argLat radians');

subplot(5,2,9)
plot(t_array,orb_elemsf(:,9),'LineWidth',1); 
axis equal; shg                     
ylabel('lonPer radians'); 
xlabel('time (s)');
title('lonPer radians'); 

subplot(5,2,10)
plot(t_array,orb_elemsf(:,10),'LineWidth',1); 
axis equal; shg                     
ylabel('p km'); 
xlabel('time (s)');
title('p semilatus rectum km');



