function [sun_vec, success] = update_sun_sensors(my_satellite_sensors, sat2sun, eclipse)
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
%   normal distribution parameters of voltage-angle dependence for each photodiode
%   mu is the mean and should be about zero, sig is standard deviation and gives noise in Volts

global const

real_n     = my_satellite_sensors.sunsensor_real_normals;
measured_n = my_satellite_sensors.sunsensor_measured_normals;
real_v     = my_satellite_sensors.sunsensor_real_voltage_maximums;
measured_v = my_satellite_sensors.sunsensor_measured_voltage_maximums;

% Assume failure or eclipse
success = 0;
sun_vec = NaN(3,1);

if (~eclipse)
    %generate voltages that would be measured by the ADCS
    [~, N] = size(real_n); % length(N) always 20
    voltages_measured = {};
    inclinometerPlots = {};
    
    if my_satellite_sensors.tag == 1 %uses Leader voltage data if Leader
        voltages_measured = my_satellite_sensors.voltages_measuredL;
        inclinometerPlots = my_satellite_sensors.inclinometerPlotsL;
    elseif my_satellite_sensors.tag == 0 %uses Follower voltage data if Follower
        voltages_measured = my_satellite_sensors.voltages_measuredF;
        inclinometerPlots = my_satellite_sensors.inclinometerPlotsF;
    end
    
    %generate voltages using real & measuresd normals and real & measured voltage maximums
    voltages = zeros(N, 1);
    thetas = zeros(N,1);
    
    for i = 1:N   
        %magnitude of a normal is the inverse of the max voltage reading
        %dividing by the frobenius norm gives a voltage measure.
        thetas(i) = acosd(sat2sun' * real_n(:, i)); %angles between [0,90]
        voltages(i) = sat2sun' * real_n(:, i) * real_v(i); %i.e. V(i) = V0cos(theta)
    end
    
    %get fit of measured data
    fitted_voltages = zeros(N,1);
    for i = 1:N
        if (1 <= i) && (i <= 4) %photodiodes 1-4; on face 1
            curvefit_m = fit(inclinometerPlots{2}',voltages_measured{i}','poly3','normalize','on');
            %plot(curvefit_m, inclinometerPlots{2}',voltages_measured{i}')
            fitted_voltages(i) = feval(curvefit_m, thetas(i));
        elseif (5 <= i) && (i <= 8)
            curvefit_m = fit(inclinometerPlots{3}',voltages_measured{i}','poly3','normalize','on');
            %plot(curvefit_m, inclinometerPlots{3}',voltages_measured{i}')
            fitted_voltages(i) = feval(curvefit_m, thetas(i));
        elseif (9 <= i) && (i <= 12)
            curvefit_m = fit(inclinometerPlots{4}',voltages_measured{i}','poly3','normalize','on');
            %plot(curvefit_m, inclinometerPlots{4}',voltages_measured{i}')
            fitted_voltages(i) = feval(curvefit_m, thetas(i));
        elseif (13 <= i) && (i <= 16)
            curvefit_m = fit(inclinometerPlots{5}',voltages_measured{i}','poly3','normalize','on');
            %plot(curvefit_m, inclinometerPlots{5}',voltages_measured{i}')
            fitted_voltages(i) = feval(curvefit_m, thetas(i));
        elseif (17 <= i) && (i <= 20)
            curvefit_m = fit(inclinometerPlots{6}',voltages_measured{i}','poly3','normalize','on');
            %plot(curvefit_m, inclinometerPlots{6}',voltages_measured{i}')
            fitted_voltages(i) = feval(curvefit_m, thetas(i));
        end
    end
    
    %i.e. funnoise = f(i,theta) + noise = V_m(i, theta) - V(i, theta) 
    funnoise = voltages - fitted_voltages; 
    
    %ignoring noise, use "funnoise" to fit to "fun"
    %i.e. fun = f(i,theta); should basically be a cosine function
    fun = fit(thetas,funnoise,'poly3','normalize','on');
    %plot(fun,thetas,funnoise)
    fitted_funvals = zeros(N,1);
    for i = 1:N
        fitted_funvals(i) = feval(fun,thetas(i));
    end
    
    noise = voltages - fitted_voltages - fitted_funvals;
    
    %normal distribution parameters of voltage-angle dependence for each photodiode
    %mu is the mean and should be about zero, sig is standard deviation and gives noise in Volts   
    [mu,sig] = normfit(noise);
    %plot(noise)
   
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
       
end
end