function [sensor_readings] = sensor_reading(my_satellite_state,other_satellite_state)
%sensor_reading returns the sensor readings

global const
sensor_readings= struct();
true_state=my_satellite_state.dynamics;


%% quaternions

[quat_ecef_eci,rate_ecef]=env_earth_attitude(true_state.time);
quat_body_eci= true_state.quat_body_eci;
quat_eci_ecef= utl_quat_conj(quat_ecef_eci);
quat_body_ecef= utl_quat_cross_mult(quat_body_eci,quat_eci_ecef);


%% gyro reading

sensor_readings.gyro_body= true_state.angular_rate_body+my_satellite_state.sensors.gyro_bias+const.gyro_noise_sdiv*randn(3,1);

%% magnetometer reading

position_ecef=utl_rotateframe(quat_ecef_eci,true_state.position_eci')';
B_ecef= env_magnetic_field(true_state.time,position_ecef);
B_body=utl_rotateframe(quat_body_ecef,B_ecef')';
sensor_readings.magnetometer_body= B_body+my_satellite_state.sensors.magnetometer_bias+const.magnetometer_noise_sdiv*randn(3,1);

%% sun sensor reading

sat2sun_eci=env_sun_vector(true_state.time);
sat2sun_body=utl_rotateframe(quat_body_eci,sat2sun_eci')';

%determine if the satellite is in eclipse with earth.
eclipse = env_eclipse(true_state.position_eci,sat2sun_eci);

[sensor_readings.sat2sun_body,...    %unit vector in the body frame
 sensor_readings.sun_sensor_true... %% true if success
 ] = update_sun_sensors(my_satellite_state.sensors, sat2sun_body, eclipse);

%% wheel angular momentum reading

sensor_readings.wheel_momentum_body= true_state.wheel_rate_body*const.JWHEEL;

%% GPS
sensor_readings.gpstime= int64(0);
sensor_readings.position_ecef= nan(3,1);
sensor_readings.velocity_ecef= nan(3,1);
sensor_readings.self2target_position_ecef= nan(3,1);
sensor_readings.target_velocity_ecef= nan(3,1);
sensor_readings.target_position_ecef= nan(3,1);
sensor_readings.target_gpstime= int64(0);

if (my_satellite_state.sensors.gps_time_till_lock<=0)
    sensor_readings.gpstime= true_state.time_ns+const.INIT_GPSNS;
    sensor_readings.position_ecef= position_ecef + randn(3,1)*const.gps_position_noise_sdiv+my_satellite_state.sensors.gps_position_bias_ecef;
    velocity_ecef= utl_rotateframe(quat_ecef_eci, true_state.velocity_eci)-cross(rate_ecef,position_ecef);
    sensor_readings.velocity_ecef= velocity_ecef + randn(3,1)*const.gps_velocity_noise_sdiv++my_satellite_state.sensors.gps_velocity_bias_ecef;
    target_position_eci= other_satellite_state.dynamics.position_eci;
    target_position_ecef= utl_rotateframe(quat_ecef_eci,target_position_eci);
    target_velocity_eci= other_satellite_state.dynamics.velocity_eci;
    target_velocity_ecef= utl_rotateframe(quat_ecef_eci, target_velocity_eci)-cross(rate_ecef,target_position_ecef);
    if (my_satellite_state.sensors.cdgps_time_till_lock<=0)
        sensor_readings.self2target_position_ecef= target_position_ecef-position_ecef+my_satellite_state.sensors.cdgps_position_bias_ecef;
    end
    % ground reading
    if (rand()<const.probability_of_ground_gps)
        sensor_readings.target_position_ecef= other_satellite_state.actuators.ground_position_ecef;
        sensor_readings.target_velocity_ecef= other_satellite_state.actuators.ground_velocity_ecef;
        sensor_readings.target_gpstime= other_satellite_state.actuators.ground_gpstime;
    end
end
end