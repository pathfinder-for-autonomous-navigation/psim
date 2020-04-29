function [sun_vec, success,mu,sig] = update_sun_sensors(my_satellite_sensors, sat2sun, eclipse)
%update_sun_sensors simulates the behavior of the sun sensor array and sun
%vector determination algorithm on the ADCS. All vectors are in the body frame.
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

global const
load fitting_coeffs

real_n     = my_satellite_sensors.sunsensor_real_normals;
measured_n = my_satellite_sensors.sunsensor_measured_normals;
real_v     = my_satellite_sensors.sunsensor_real_voltage_maximums;
measured_v = my_satellite_sensors.sunsensor_measured_voltage_maximums;

% Assume failure or eclipse
success = 0;
sun_vec = NaN(3,1);

if (~eclipse)
    [~, N] = size(real_n); % length(N) always 20
    
    %psim generated voltages using real & measuresd normals and real & measured voltage maximums
    %i.e.voltages that would be measured by the ADCS
    voltages_generated = zeros(N, 1);
    thetas_generated = zeros(N,1);
    
    for i = 1:N   
        %magnitude of a normal is the inverse of the max voltage reading
        %dividing by the frobenius norm gives a voltage measure.
        angle = acosd(sat2sun' * real_n(:, i)); %corresponding angle  between [0,180]
        if angle >= 70
            thetas_generated(i) = NaN; %throw out angles outside of range [0,70]
        else
            thetas_generated(i) = angle;
        end       
        voltages_generated(i) = sat2sun' * real_n(:, i) * real_v(i); %i.e. V(i) = V0cos(theta)
    end
    
    %voltages measured by sun sensors (Josh's empirical data)
    [~, N] = size(real_n); % length(N) always 20
    
    if my_satellite_sensors.tag == 1 %uses Leader voltage data if Leader
        coeffs = Cs_L;
    elseif my_satellite_sensors.tag == 0 %uses Follower voltage data if Follower
        coeffs = Cs_F;
    end
    
    fitted_voltages = zeros(20,1); %get fitted voltages using the nonlinear fitting
    
    for i = 1:N
        if i == 1
            Cs = coeffs{2,1};
        elseif i == 2
            Cs = coeffs{2,2};
        elseif i == 3
            Cs = coeffs{2,3};
        elseif i == 4
            Cs = coeffs{2,4};
        elseif i == 5
            Cs = coeffs{3,1};
        elseif i == 6
            Cs = coeffs{3,2};
        elseif i == 7
            Cs = coeffs{3,3};
        elseif i == 8
            Cs = coeffs{3,4};
        elseif i == 9
            Cs = coeffs{4,1};
        elseif i == 10
            Cs = coeffs{4,2};
        elseif i == 11
            Cs = coeffs{4,3};
        elseif i == 12
            Cs = coeffs{4,4};
        elseif i == 13
            Cs = coeffs{5,1};
        elseif i == 14
            Cs = coeffs{5,2};
        elseif i == 15
            Cs = coeffs{5,3};
        elseif i == 16
            Cs = coeffs{5,4};
        elseif i == 17
            Cs = coeffs{6,1};
        elseif i == 18
            Cs = coeffs{6,2};
        elseif i == 19
            Cs = coeffs{6,3};
        elseif i == 20
            Cs = coeffs{6,4};
        end
            
        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas)(Cs(1) + (Cs(2)*cos(thetas)) + ...
                            Cs(3)*thetas.^2 +...
                            (Cs(4)*thetas.^4));
        
        %these fitted_voltages are called for psim model
        fitted_voltages(i) = scaled_fun(Cs,thetas_generated(i));

%         angles = linspace(min(thetas_measured),max(thetas_measured));
%         plot(thetas_measured,voltages,'ko',angles,fun(Cs,angles),'b-')
%         legend('Data','fitted function'); 
%         xlabel('theta (degrees)'); ylabel('voltages (V)')
%        title(['fitted function for measured voltages (Joshs data) #',int2str(i)]);
%         saveas(gcf,['data_and_fits' int2str(i) '.png'])
        
    end
    
%     figure(2)
%     scatter(thetas_generated, voltages_generated)
%     xlabel('theta (degrees)'); ylabel('voltages (V)')
%     title('voltages generated');
%     saveas(gcf,'generated_voltages.png')
%         
%     figure(3)
%     scatter(thetas_generated, fitted_voltages)
%     xlabel('theta (degrees)'); ylabel('voltages (V)')
%     title('fitted voltages');
%     saveas(gcf,'fitted_voltages.png')
    
%    figure(4)
    %voltages_generated = v  - fitted_voltages;
    voltages_generated = fitted_voltages;
    
%     scatter(thetas_generated,voltages_generated)
%     xlabel('theta (degrees)'); ylabel('voltages (V)')
%     title('redefined_generated_voltages');
%     saveas(gcf,'redefined_generated_voltages.png')
    
    %normal distribution parameters of voltage-angle dependence for each photodiode
    %mu is the mean and should be about zero, sig is standard deviation and gives noise in Volts   
    [mu,sig] = normfit(voltages_generated);
    
    %%% trimming algorithm
    S = zeros(N, 3);
    v = zeros(N, 1); 
    thresh = max(voltages_generated)*0.5;
    j = 0;
    for i = 1:N
        %enforce voltage threshold for inclusion in sun vector calculation
        if voltages_generated(i) > thresh
            j = j + 1;
            S(j, :) = measured_n(:, i)';
            v(j) = voltages_generated(i);
        end
    end
   
    %%%% least squares: takes in S, v to output sun_vector
    %run sun vector determination algorithm run by the adcs computer
    if j >= 3  % larger j --> smaller residuals
        S = S(1:j,:);
        v = v(1:j,:);
        [Q, R] = qr(S);
        sun_vec = R \ (Q' * v);
        sun_vec = sun_vec / norm(sun_vec);
        success = 1;
    end
    
end
end