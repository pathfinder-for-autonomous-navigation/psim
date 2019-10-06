function [sensor_readings] = sensor_reading(sensor_state,true_state,actuators)
%sensor_reading returns the sensor readings
%   sensor_readings is a struct with elements:
%       gyro_body, gyro reading (rad/s)
%       magnetometer_body, magnetometer reading (T)
%       sat2sun_body, unit vector from satellite to sun.
%       sun_sensor_true, true if sun vector reading is good, else false.
%       wheel_momentum_body, wheel angular momentum reading (Nms)
%       time, time since inital GPS week (s)
%       position_ecef, position of the gps reciever of the satellite.
%   true_state is a struct with elements:
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
%   TODO implement the actual sensors with errors
%   TODO implement GPS
global const
%% quaternions
[quat_ecef_eci,~]=env_earth_attitude(true_state.time);
quat_ecef_eci= utl_array2quaternion(quat_ecef_eci);
quat_body_eci= utl_array2quaternion(true_state.quat_body_eci);
quat_eci_ecef= conj(quat_ecef_eci);
quat_body_ecef=quat_eci_ecef*quat_body_eci;
%% gyro reading
sensor_readings.gyro_body= true_state.angular_rate_body;
%% magnetometer reading
position_ecef=rotateframe(quat_ecef_eci,true_state.position_eci')';
B_ecef= env_magnetic_field(true_state.time,position_ecef);
sensor_readings.magnetometer_body=rotateframe(quat_body_ecef,B_ecef')';
%% sun sensor reading
sat2sun_eci=env_sun_vector(true_state.time);
sat2sun_body=rotateframe(quat_body_eci,sat2sun_eci')';
%now determine if the satellite is in eclipse with earth.
eclipse = env_eclipse(true_state.position_eci,sat2sun_eci);
sun_in_dead_zone=(sat2sun_body(3))>cos(const.SUNSENSOR_DEADZONE);
%we can't read the sun sensor because the sun is in the dead zone
if ((~eclipse) && (~sun_in_dead_zone))
    sensor_readings.sat2sun_body=rotateframe(quat_body_eci,sat2sun_eci')';
    sensor_readings.sun_sensor_true= true;
else
    sensor_readings.sat2sun_body=[0;0;0;];
    sensor_readings.sun_sensor_true= false;
end
%% wheel angular momentum reading
sensor_readings.wheel_momentum_body= true_state.wheel_rate_body*const.JWHEEL;
%% GPS
sensor_readings.time= true_state.time;
sensor_readings.position_ecef= rotateframe(quat_ecef_eci,true_state.position_eci')';
%TODO get velocity in ECEF
end

