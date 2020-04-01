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
 sensor_readings.sun_sensor_true,... %true if succesfull
] = update_sun_sensors(my_satellite_state.sensors, sat2sun_body, eclipse);

%% wheel angular momentum reading

sensor_readings.wheel_momentum_body= true_state.wheel_rate_body*const.JWHEEL;

%% GPS
sensor_readings.time= 0;
sensor_readings.position_ecef= nan(3,1);
sensor_readings.velocity_ecef= nan(3,1);
sensor_readings.self2target_position_ecef= nan(3,1);
sensor_readings.target_velocity_ecef= nan(3,1);
sensor_readings.target_position_ecef= nan(3,1);
sensor_readings.target_time= 0;

if (my_satellite_state.sensors.gps_time_till_lock<=0)
    sensor_readings.time= true_state.time;
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
        sensor_readings.target_time= other_satellite_state.actuators.ground_time;
    end
end


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
    [~, N] = size(real_n); % I'm assuming N between [2,4]
    voltages = zeros(N, 1);
    
    if N == 2
        %assuming 2 sun sensors are reading full voltage
    elseif N == 3
        %assuming 3 sun sensors are reading full voltage
    elseif N == 4
        %assuming 4 sun sensors are reading full voltage
        %reads Josh's sun sensor log to get voltage-angle relations for ONE FACE
        %we need to use the mu and sig values somehow to scale/offset the 'voltages' vector
        [V_Photo0_trim, V_Photo1_trim, V_Photo2_trim, V_Photo3_trim, mus, sigs] = read_log();
        for i = 1:N   
            %magnitude of a normal is the inverse of the max voltage reading so
            %dividing by the frobenius norm gives a voltage measure.
            voltages(i) = sat2sun' * real_n(:, i) * real_v(i) * sigs(i); 
       end
    end
    
   
    %%% trimming algorithm
    S = zeros(N, 3);
    v = zeros(N, 1);
    thresh = max(voltages)*0.5;
    j = 0;
    for i = 1:N
        %enforce voltage threshold for inclusion in sun vector calculation
        voltages(i) = voltages(i) / measured_v(i);
        if voltages(i) > thresh
            j = j + 1;
            S(j, :) = measured_n(:, i)';
            v(j) = voltages(i);
        end
    end
    
    %%%% least squares
    %run sun vector determination algorithm run by the adcs computer
    %%% takes in S, v to output sun_vector
    if j >= 3  % TODO : Perhaps play with this parameter; larger j --> smaller residuals
        S = S(1:j, :);
        v = v(1:j, :);
        [Q, R] = qr(S);
        sun_vec = R \ (Q' * v);
        sun_vec = sun_vec / norm(sun_vec);
        success = 1;
    end
    
    %%%% convert sun_vector into spherical coords
    %%%% get error of sun vector in spherical coords
    
end
end

function [V_Photo0_trim, V_Photo1_trim, V_Photo2_trim, V_Photo3_trim, mus, sigs] = read_log()
    %read Josh's sun sensor logs to get voltage-angle relations
    
    %open voltage log data for ONE FACE
    fid = fopen('Face3-YLeader16204804','r');
    tline = fgetl(fid);
    
    while ischar(tline)
        A{length(A)+1} = tline;
        tline = fgetl(fid);
    end
    fclose(fid);
    
    for k = 1:length(A)-1
        j = 1;
        while A{k}(j) ~= '='
            j = j+1;
        end
        A{k} = A{k}(j+1:end);
    end
    
    inclinometerX = [];
    inclinometerY = [];
    V_Photo0 = [];
    V_Photo1 = [];
    V_Photo2 = [];
    V_Photo3 = [];
    for l = 1:6:length(A)
        inclinometerX(length(inclinometerX)+1) = str2double(A{l});
    end
    for l = 2:6:length(A)
        inclinometerY(length(inclinometerY)+1) = str2double(A{l});
    end
    for m = 3:6:length(A)
        V_Photo0(length(V_Photo0)+1) = str2double(A{m});
    end
    for m = 4:6:length(A)
        V_Photo1(length(V_Photo1)+1) = str2double(A{m});
    end
    for m = 5:6:length(A)
        V_Photo2(length(V_Photo2)+1) = str2double(A{m});
    end
    for m = 6:6:length(A)
        V_Photo3(length(V_Photo3)+1) = str2double(A{m});
    end
    
    %select data range
    figure
    plot(V_Photo0)
    title('Click on Left and Right to Choose X Bounds')
    [x,y] = ginput(2);
    x = sort(x);
    close
    x = floor(x);
    
    % trim data to range
    inclinometerX_trim = inclinometerX(x(1):x(2));
    inclinometerY_trim = inclinometerY(x(1):x(2));
    V_Photo0_trim = V_Photo0(x(1):x(2));
    V_Photo1_trim = V_Photo1(x(1):x(2));
    V_Photo2_trim = V_Photo2(x(1):x(2));
    V_Photo3_trim = V_Photo3(x(1):x(2));
    
    % apply low pass filter
    V_Photo0_filt = V_Photo0_trim;
    V_Photo1_filt = V_Photo1_trim;
    V_Photo2_filt = V_Photo2_trim;
    V_Photo3_filt = V_Photo3_trim;
    
    inclinometerX_filt = inclinometerX_trim;
    inclinometerY_filt = inclinometerY_trim;
    a = 0.92;
    for i = 2:length(V_Photo0_filt)
        V_Photo0_filt(i) = V_Photo0_filt(i-1)*a + V_Photo0_filt(i)*(1-a);
        V_Photo1_filt(i) = V_Photo1_filt(i-1)*a + V_Photo1_filt(i)*(1-a);
        V_Photo2_filt(i) = V_Photo2_filt(i-1)*a + V_Photo2_filt(i)*(1-a);
        V_Photo3_filt(i) = V_Photo3_filt(i-1)*a + V_Photo3_filt(i)*(1-a);
        inclinometerY_filt(i) = inclinometerY_filt(i-1)*a + inclinometerY_filt(i)*(1-a);
        inclinometerX_filt(i) = inclinometerX_filt(i-1)*a + inclinometerX_filt(i)*(1-a);
    end    
    
    %get angle plot (y-axis angle)
    if file(1) == 'B'
        inclinometerPlot = inclinometerX_trim;
    else
        inclinometerPlot = inclinometerY_trim;
    end
   
    % voltage-angle plot
    mus = zeros(4,1);
    sigs = zeros(4,1);
    
    for h = 0:3
        %x-axis angle in degrees, y-axis voltage in V
        eval(['scatter(inclinometerPlot,V_Photo' num2str(h) '_trim,''k.'')']);
        [mu,sig] = normfit(inclinometerPlot, 'V_Photo1_trim');
        mus(h) = mu; sigs(h) = sig;
    end

end
