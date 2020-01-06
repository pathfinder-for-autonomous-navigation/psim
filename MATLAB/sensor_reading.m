function [sensor_readings] = sensor_reading(my_satellite_state,other_satellite_state)
%sensor_reading returns the sensor readings
%   TODO implement the actual sensors with errors
%   TODO implement GPS and CDGPS
%   TODO use get_truth function.

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
 sensor_readings.sun_sensor_true,... %true if succesfull
] = update_sun_sensors(my_satellite_state.sensors, sat2sun_body, eclipse);

%% wheel angular momentum reading

sensor_readings.wheel_momentum_body= true_state.wheel_rate_body*const.JWHEEL;

%% GPS

sensor_readings.time= true_state.time;
sensor_readings.position_ecef= position_ecef;
sensor_readings.velocity_ecef= utl_rotateframe(quat_ecef_eci, true_state.velocity_eci)-cross(rate_ecef,position_ecef);


%in reality this is not how cd cps works
target_position_eci= other_satellite_state.dynamics.position_eci;
sensor_readings.target_position_ecef= utl_rotateframe(quat_ecef_eci,target_position_eci);
target_velocity_eci= other_satellite_state.dynamics.velocity_eci;
sensor_readings.target_velocity_ecef= utl_rotateframe(quat_ecef_eci, target_velocity_eci)-cross(rate_ecef,sensor_readings.target_position_ecef);

end


function [sun_vec, success] = update_sun_sensors(my_satellite_sensors, sat2sun, eclipse)
%update_sun_sensors simulates the behavior of the sun sensor array and sun
%vector determination algorithm on the ADCS. All vectors are in the body
%frame.
%   my_satellite_sensors is the struct containing all state information
%       about the satellites sensors (for our purposes here the real
%       normals, measured normals, real voltage maximums, and measured
%       voltage maximums.
%   sat2sun is a unit vector pointing from the satellite to the sun in the
%       body frame.
%   eclipse is true if the satellite is in eclipse and false otherwise.
%   sun_vec is the measured sun vector in the body frame of the spacecraft.
%   success is true if the returned sun vector is accurate and false
%       otherwise.

real_n     = my_satellite_sensors.sunsensor_real_normals;
measured_n = my_satellite_sensors.sunsensor_measured_normals;
real_v     = my_satellite_sensors.sunsensor_real_voltage_maximums;
measured_v = my_satellite_sensors.sunsensor_measured_voltage_maximums;

% Assume failure or eclipse
success = 0;
sun_vec = NaN(3,1);

if (~eclipse)

    %generate voltages that would be measured by the ADCS
    [~, N] = size(real_n);
    voltages = zeros(N, 1);
    for i = 1:N
        %magnitude of a normal is the inverse of the max voltage reading so
        %dividing by the frobenius norm gives a voltage measure.
        voltages(i) = sat2sun' * real_n(:, i) * real_v(i);
    end

    %run sun vector determination algorithm run by the adcs computer
    A = zeros(N, 3);
    Y = zeros(N, 1);
    j = 0;
    for i = 1:N
        %enforce voltage threshold for inclusion in sun vector calculation
        voltages(i) = voltages(i) / measured_v(i);
        if voltages(i) > 0.5
            j = j + 1;
            A(j, :) = measured_n(:, i)';
            Y(j) = voltages(i);
        end
    end
    if j >= 3  % TODO : Perhaps play with this parameter
        A = A(1:j, :);
        Y = Y(1:j, :);
        [Q, R] = qr(A);
        sun_vec = R \ (Q' * Y);
        sun_vec = sun_vec / norm(sun_vec);
        success = 1;
    end
    
end
end
