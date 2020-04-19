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
        thetas_generated(i) = acosd(sat2sun' * real_n(:, i)); %corresponding angle  between [0,90]
        voltages_generated(i) = sat2sun' * real_n(:, i) * real_v(i); %i.e. V(i) = V0cos(theta)
    end
    
    %voltages measured by sun sensors (Josh's empirical data)
    [~, N] = size(real_n); % length(N) always 20
    voltages_measured = {};
    inclinometerPlots = {};
    
    if my_satellite_sensors.tag == 1 %uses Leader voltage data if Leader
        voltages_measured = my_satellite_sensors.voltages_measuredL;
        inclinometerPlots = my_satellite_sensors.inclinometerPlotsL;
        PCBnorms = my_satellite_sensors.normsL(:); %turn into collumn vector
    elseif my_satellite_sensors.tag == 0 %uses Follower voltage data if Follower
        voltages_measured = my_satellite_sensors.voltages_measuredF;
        inclinometerPlots = my_satellite_sensors.inclinometerPlotsF;
        PCBnorms = my_satellite_sensors.normsF(:); %turn into collumn vector
    end
    
%     %%% get offset angles off major axis
%     psi = zeros(N,1);
%     for k = 1:N
%         v = PCBnorms{i};
%         %psi(k) = acosd(dot(v,real_n(:, k))/(norm(v)*norm(real_n(:, k))));
%         psi(k) = acosd(dot(v,sat2sun')/(norm(v)*norm(sat2sun')));
%     end
    
    fitted_voltages = zeros(20,1); %get fitted voltages using the nonlinear fitting
    
    for i = 1:N
        %each face has a different inclinometer plot
        if (i >=1) && (i <= 4) 
            thetas_measured = inclinometerPlots{2};
        elseif (i >=5) && (i <= 8)
            thetas_measured = inclinometerPlots{3};
        elseif (i >=9) && (i <= 12)
            thetas_measured = inclinometerPlots{4};
        elseif (i >=13) && (i <= 16)
            thetas_measured = inclinometerPlots{5};
        elseif (i >=17) && (i <= 20)
            thetas_measured = inclinometerPlots{6};
        end
        
        %redefine thetas measured
        %thetas_measured = thetas_measured-psi(i);
        
        %using the measured voltages, do a nonlinear fitting
        voltages = voltages_measured{i};
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^1 + Cs(4)*thetas_measured.^2 +...
                            Cs(5)*thetas_measured.^4);
                        
        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.1,0.01,0.1,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + 3.3*(Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^1 + Cs(4)*thetas_measured.^2 +...
                            (Cs(5)*thetas_measured.^4));
        
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
    voltages_generated = voltages_generated - fitted_voltages;
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
    newPCBnorms = zeros(N,3);
    
    thresh = max(voltages_generated)*0.5;
    j = 0;
    for i = 1:N
        %enforce voltage threshold for inclusion in sun vector calculation
        if voltages_generated(i) > thresh
            j = j + 1;
            S(j, :) = measured_n(:, i)';
            v(j) = voltages_generated(i);
            newPCBnorms(j,:) = PCBnorms{i}'; %prep for matrix mult
        end
    end
    
    x = 'test';
    alphas = zeros(j,1);
    newVolts = zeros(j,1);
    %%% get rotation off of major axis
    if j >= 3  %larger j --> smaller residuals           
        S = S(1:j, :); %psim generated
        newPCBnorms = newPCBnorms(1:j,:); 
        for i = 1:j
            alphas(i) = acosd(mtimes(S(i)', newPCBnorms(i)));
            newVolts(i) = mtimes(S(i)', newPCBnorms(i));
        end
    end
    
    x = 'test';
    
    %%%% least squares
    %run sun vector determination algorithm run by the adcs computer
    %%% takes in S, v to output sun_vector
    if j >= 3  % larger j --> smaller residuals
        S = S(1:j, :);
        v = v(1:j, :);
        [Q, R] = qr(S);
        sun_vec = R \ (Q' * v);
        sun_vec = sun_vec / norm(sun_vec);
        success = 1;
    end
    
end
end