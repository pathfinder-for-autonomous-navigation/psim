%Team Bread, Sruti Vutukury, Anthony Ordonez, Michael Zakoworotny
%Preliminary Report

%%%constants
g_t = 9.81; %gravity Earth
mu_t = 1.48*10^-5; %viscosity Earth
rho_t = 1.225; %air density Earth
sos_t = 344; %speed of sound Earth

%%%%%%%% EG (extraterrestrial glider) %%%%%%%%%%%%%%%%
g_e = g_t/10; %gravity exoplanet
rho_e = rho_t/200; %density exoplanet
mu_e = mu_t/1.5; %viscosity exoplanet
sos_e = 120; %speed of sound exoplanet
m_e = 25; %fixed variable; max mass of EG
Cl_max = 1.5; %max coefficient of lift
ClCdratio = 29; %fixed; 
b_e = 6; %m span
AR_e = 12; %fixed 
S_e = (b_e^2)/AR_e; %m^2 area
c_e = S_e/b_e; %m chord

weight_e = m_e*g_e;
lift_e = weight_e;
drag_e = lift_e/ClCdratio;
v_e = sqrt((2*lift_e)/(Cl_max*rho_e*S_e)); %m/s average cruise velocity
Re_e = rho_e*v_e*c_e*mu_e^-1; %cruise Re number
Ma_e = v_e/sos_e; %cruise Mach number

%%%%%%%% TG (terrestrial glider) %%%%%%%%%%%%%%%%
Re_t = Re_e;
Ma_t = Ma_e;
AR_t = AR_e;

syms v_t c_t b_t S_t
eqns = [Ma_t - v_t/sos_t == 0, Re_t - rho_t*v_t*c_t*mu_t^-1 == 0,...
    AR_t - (b_t^2/S_t) == 0, S_t - (b_t*c_t) == 0];
T = solve(eqns, [v_t c_t b_t S_t]);
v_t = double(T.v_t);
c_t = double(T.c_t);
b_t = double(T.b_t);
S_t = double(T.S_t);
lift_t = (1/2)*Cl_max*rho_t*v_t^2*S_t;
drag_t = lift_t/ClCdratio;
weight_t = lift_t;
m_t = weight_t/g_t;




