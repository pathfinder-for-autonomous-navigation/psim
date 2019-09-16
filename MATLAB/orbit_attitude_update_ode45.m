function [final_state] = orbit_attitude_update_ode45(initial_state,actuators,delta_time)
%orbit_attitude_update updates the ECEF position, velocity, and attitude of
%the satellite. (units are all MKS)
%   initial_state and final state are a structs with elements:
%       time, time since inital GPS week.
%       position_eci, position of the center of mass of the satellite.
%       velocity_eci, velocity of the center of mass of the satellite.
%       angular_rate_body, the angular rate of the spacecraft in the body frame.
%       quat_body_eci, quaternion that rotates from eci to body frame.
%       wheel_rate_body, x,y, and z, wheel angular rates.
%       fuel_net_angular_momentum_eci, net angular momentum of the fuel.
%       fuel_mass, the mass of the fuel.
%   actuators is a struct with actuator inputs that are constant over the
%   following time step but not constant for the whole simulation:
%       firing_start_times, times since inital GPS week to start firing.
%       real_thrust_vectors_body, real thruster forces, units N.
%       centers_of_thrust_body, center of thrust for each firing, units m.
%       firing_on_times, how long firings last.
%       wheel_commanded_rate, commanded x,y,z wheel rate.
%       wheel_commanded_ramp, commanded x,y,z wheel ramp, units rad/s/s.
%       magrod_real_moment_body, real magnetorquer moment, units A*m^2
%
global const
    function rate= wheelrate(t)
        %wheel rate at any given time, includes saturation
        real_ramp= min(abs(actuators.wheel_commanded_ramp),const.MAXWHEELRAMP);%saturate ramp
        real_commanded_rate=max(min(actuators.wheel_commanded_rate,const.MAXWHEELRATE),-const.MAXWHEELRATE);%saturate rate
        real_ramp= sign(actuators.wheel_commanded_rate-initial_state.wheel_rate_body).*real_ramp;%determine ramp direction
        target_rate=zeros([3,1]);
        for i= 1:3
            if real_ramp(i)==0
                target_rate(i)=initial_state.wheel_rate_body(i);
            elseif real_ramp(i)<0
                target_rate(i)= max(initial_state.wheel_rate_body(i)+real_ramp(i)*(t-initial_state.time),real_commanded_rate(i));
            else
                target_rate(i)= min(initial_state.wheel_rate_body(i)+real_ramp(i)*(t-initial_state.time),real_commanded_rate(i));
            end
        end
        rate= max(min(target_rate,const.MAXWHEELRATE),-const.MAXWHEELRATE);
    end
    function ramp= wheelramp(t)
        %wheel ramp at any given time, includes saturation
        real_ramp= min(abs(actuators.wheel_commanded_ramp),const.MAXWHEELRAMP);
        real_commanded_rate=max(min(actuators.wheel_commanded_rate,const.MAXWHEELRATE),-const.MAXWHEELRATE);
        ramp= sign(real_commanded_rate-wheelrate(t)).*real_ramp;
    end
    function force= thrust_force(t)
        %thrust force at time t in the body frame
        %TODO add actual thrust
        force=0.0;
    end
    function torque= thrust_torque(t)
        %thrust torque at time t in the body frame
        %TODO add actual thrust
        torque=0.0;
    end
    function mass= fuel_mass(t)
        %fuel_mass at time t
        %TODO add actual fuel change
        mass=initial_state.fuel_mass;
    end
    [quat_ecef_eci_initial,~]=env_earth_attitude(initial_state.time);
    [quat_ecef_eci_final,~]=env_earth_attitude(initial_state.time+delta_time);
    quat_ecef_eci_initial= utl_array2quaternion(quat_ecef_eci_initial);
    quat_ecef_eci_final= utl_array2quaternion(quat_ecef_eci_final);
    function quat_ecef_eci= earth_first_order_rotation(t)
        %returns earths attitude at time t, using a first order
        %approximation (slerp)
        %of the rotation from the start to end of delta_time
        %disp((t-initial_state.time)/delta_time)
        T=min((t-initial_state.time)/delta_time,1.0);
        quat_ecef_eci= slerp(quat_ecef_eci_initial,quat_ecef_eci_final,T);
    end
    magnetic_field_zero_order=env_magnetic_field(initial_state.time,rotateframe(quat_ecef_eci_initial,initial_state.position_eci')');
    function dydt= state_dot(t,y)
        %y = [x; y; z; xdot; ydot; zdot;q1;q2;q3;q4;ratex;ratey;ratez;
        %     1  2  3  4     5     6    7  8  9  10 11    12    13    
        %    
        %       fuel_ang_momentx;fuel_ang_momenty;fuel_ang_momentz;]
        %       14               15               16
        %
        % Don't have any super expensive calculation in here.
        %   Instead make first or zero order approximations of
        %   perturbations before, and pass in as a function of time.
        
        dydt=zeros([16,1]);
        dydt(1:3)=y(4:6);
        quat_ecef_eci= earth_first_order_rotation(t);
        quat_eci_ecef= conj(quat_ecef_eci);
        quat_body_eci=utl_array2quaternion(y(7:10));
        quat_eci_body= conj(quat_body_eci);
        quat_body_ecef=quat_eci_ecef*quat_body_eci;
        pos_eci=y(1:3);
        pos_ecef=rotateframe(quat_ecef_eci,pos_eci')';
        [g_ecef,~,G_ecef]= env_gravity(t,pos_ecef);
        g_eci=rotateframe(quat_eci_ecef,g_ecef')';
        %TODO add thruster firings forces and torques
        %TODO add drag and solar pressure forces
        dydt(4:6)= g_eci;
        quat_rate=quaternion(0,y(11),y(12),y(13));
        dydt(7:10)= utl_quaternion2array(0.5*quat_body_eci*quat_rate);
        Lwb= const.JWHEEL*wheelramp(t);
        %calculation of external torques 
        %TODO add drag, solar pressure, and gravity torques
        magnetic_field_body=rotateframe(quat_body_ecef,magnetic_field_zero_order')';
        magnetic_torque_body= cross(actuators.magrod_real_moment_body,magnetic_field_body);
        %calculate the difference in the rates of the fuel and the sat
        Jfuel= const.JFUEL_NORM*fuel_mass(t);%moment of inertia of fuel, spherical
        rate_fuel_eci= y(14:16)/Jfuel;
        rate_sat_eci= rotateframe(quat_eci_body,y(11:13)')';
        torque_from_fuel_eci= const.SLOSH_DAMPING*(rate_fuel_eci-rate_sat_eci);
        dydt(14:16)= -torque_from_fuel_eci;
        Lb=magnetic_torque_body+rotateframe(quat_body_eci,torque_from_fuel_eci')';
        dydt(11:13)=const.JBINV*( Lb-Lwb-cross(y(11:13),const.JB*y(11:13)+const.JWHEEL*wheelrate(t)));
        %dydt(11:13)=const.JBINV*(-cross(y(11:13),const.JB*y(11:13)));
        %equation 3.147 in Fundamentals of Spacecraft Attitude Determination and Control
    end
    init_y=zeros([16,1]);
    init_y(1:3)=initial_state.position_eci;
    init_y(4:6)=initial_state.velocity_eci;
    init_y(7:10)=initial_state.quat_body_eci;
    init_y(11:13)=initial_state.angular_rate_body;
    init_y(14:16)=initial_state.fuel_net_angular_momentum_eci;
    final_time=initial_state.time+delta_time;
    opts = odeset('RelTol',1E-12,'AbsTol',1e-9);
    [~,ys]=ode45(@state_dot,[initial_state.time,final_time],init_y,opts);
    %ys=utl_ode2(@state_dot,[initial_state.time,final_time],init_y);
    final_y= ys(end,:)';
    final_state=initial_state;
    final_state.time=final_time;
    final_state.position_eci=final_y(1:3);
    final_state.velocity_eci=final_y(4:6);
    final_state.quat_body_eci=final_y(7:10)/norm(final_y(7:10));
    final_state.angular_rate_body=final_y(11:13);
    final_state.fuel_net_angular_momentum_eci=final_y(14:16);
    final_state.wheel_rate_body=wheelrate(final_time);
    final_state.fuel_mass=fuel_mass(final_time);
end



