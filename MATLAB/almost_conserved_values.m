function [orbital_energy,orbital_angular_momentum_eci,rotational_energy,spacecraft_angular_momentum_eci] = almost_conserved_values(state)
%orbital_energy returns the orbital kinetic + potential energy 
%   units J
%   state is a structs with elements:
%       time, datetime time zone UTC leepseconds.
%       position_eci, position of the center of mass of the satellite.
%       velocity_eci, velocity of the center of mass of the satellite.
%       fuel_mass, the mass of the fuel.
global const
[q,~]=env_earth_attitude(state.time);
quat_ecef_eci=utl_array2quaternion(q);
[~,PG,~]= env_gravity(state.time,rotateframe(quat_ecef_eci,state.position_eci')');
PE=-PG*(const.MASS+state.fuel_mass);
KE= 0.5*(state.fuel_mass+const.MASS)*state.velocity_eci'*state.velocity_eci;
orbital_energy= KE+PE;
orbital_angular_momentum_eci=cross(state.position_eci,state.velocity_eci)*(const.MASS+state.fuel_mass);
quat_body_eci= utl_array2quaternion(state.quat_body_eci);
quat_eci_body= conj(quat_body_eci);
hb_body= const.JB*state.angular_rate_body+const.JWHEEL*state.wheel_rate_body;
rotational_energy= state.angular_rate_body'*const.JB*state.angular_rate_body;
rotational_energy= rotational_energy+state.wheel_rate_body'*const.JWHEEL*state.wheel_rate_body;
Jfuel= const.JFUEL_NORM*state.fuel_mass;
rotational_energy= rotational_energy+state.fuel_net_angular_momentum_eci'*inv(Jfuel)*state.fuel_net_angular_momentum_eci;
rotational_energy= 0.5*rotational_energy;
spacecraft_angular_momentum_eci= rotateframe(quat_eci_body,hb_body')'+state.fuel_net_angular_momentum_eci;
end

