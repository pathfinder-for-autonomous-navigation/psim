%%%%%% gyro analysis
%%%%%% last editted Sruti Vutukury 3/27/2020

%%%ETU test Stewart's house 3/21/2020
%uses continuous temperature recorded by satellite; need to calibrate
data = csvread('gyro_bias_stability_3-21-20.csv',1,0);

%time in ms
time = data(:,1);

%mag data in Teslas
mag1x = data(:,2); 
mag1y = data(:,3);
mag1z = data(:,4); 

%gyro data multiplied by 0.5 to correct for an error in driver script. driver script has since been corrected 
%gyro data in rad/sec
gyrox = data(:,5).*0.5; %set with outliers; removed earlier
gyroy = data(:,6).*0.5;
gyroz = data(:,7).*0.5;
temp =  data(:,8); %satellite recorded temp (uncalibrated)

%convert to gyro readings to deg/sec for some calculations
gyroxd = gyrox.*(180/pi); 
gyroyd = gyroy.*(180/pi);
gyrozd = gyroz.*(180/pi);

Fs = 50; %sampling frequency 50 hz
T  = 1/Fs; %sampling time
tts = time(length(time))-time(1)/1000; %total time in seconds
tth = time(length(time))-time(1)/(1000*3600); %total time in hours

%% get basic stats about gyro/mag

diffs = ones(6,1); %peak-to-peak values
means = ones(6,1);
stdevs = ones(6,1);
RMSvals = ones(6,1); %root mean square values
avgtemp = mean(temp(:)); %average temperature in data collection interval

for j = 1:6
    if j == 1 
        diffs(j) = max(mag1x) - min(mag1x);
        means(j) = mean(mag1x);
        stdevs(j) = std(mag1x);
        RMSvals(j) = rms(mag1x);
    elseif j == 2
        diffs(j) = max(mag1y) - min(mag1y);
        means(j) = mean(mag1y);
        stdevs(j) = std(mag1y);
        RMSvals(j) = rms(mag1y);
    elseif j == 3
        diffs(j) = max(mag1z) - min(mag1z);
        means(j) = mean(mag1z);
        stdevs(j) = std(mag1z);
        RMSvals(j) = rms(mag1z);
    elseif j == 4
        diffs(j) = max(gyrox) - min(gyrox);
        means(j) = mean(gyrox);
        stdevs(j) = std(gyrox);
        RMSvals(j) = rms(gyrox);
    elseif j == 5
        diffs(j) = max(gyroy) - min(gyroy);
        means(j) = mean(gyroy);
        stdevs(j) = std(gyroy);
        RMSvals(j) = rms(gyroy);
    elseif j == 6
        diffs(j) = max(gyroz) - min(gyroz);
        means(j) = mean(gyroz);
        stdevs(j) = std(gyroz);
        RMSvals(j) = rms(gyroz);
    end
end

%% Angular rate zero-rate level
%calculated as the mean of the signal
%zero rate level when there is no angular velocity applied to the gyroscope
%source: https://sst.semiconductor-digest.com/2010/11/introduction-to-mems-gyroscopes/
%https://learn.adafruit.com/comparing-gyroscope-datasheets/overview

%measured in deg/sec
ARZRLx = mean(means(4))*(180/pi);
ARZRLy = mean(means(5))*(180/pi);
ARZRLz = mean(means(6))*(180/pi);

%% gyroscope bias instability
%source: https://www.vectornav.com/support/library/gyroscope

%get bias stability in deg/sec
[avarx,taux] = allanvar(gyroxd); adevx = sqrt(avarx);
[avary,tauy] = allanvar(gyroyd); adevy = sqrt(avary);
[avarz,tauz] = allanvar(gyrozd); adevz = sqrt(avarz);

%bias stability in deg/s
bias_stabx = min(adevx); 
bias_staby = min(adevy);
bias_stabz = min(adevz); 

%allan variance plot
figure(1)
loglog(taux,adevx)
hold on
loglog(tauy,adevy)
loglog(tauz,adevz)
xline(1);
xlabel('tau [s]'); ylabel('sigma(tau) [deg/s]')
title('Allan Variance')
legend('gyrox','gyroy','gyroz')
grid on

%% Angle Random Walk
%angle random walk in deg/s^(3/2)
%value at a sampling time of 1 second on the Allan Variance plot 
angle_rand_walkx = interp1(taux,adevx,1,'spline'); 
angle_rand_walky = interp1(tauy,adevy,1,'spline');
angle_rand_walkz = interp1(tauz,adevz,1,'spline');

%% Gyro Noise Density
%source: https://ez.analog.com/mems/w/documents/4507/faq-gyroscope-noise-density
[freqx,PSD2x,RMSx,RNDx] = ProcessNoise(gyroxd,50); 
[freqy,PSD2y,RMSy,RNDy] = ProcessNoise(gyroyd,50);
[freqz,PSD2z,RMSz,RNDz] = ProcessNoise(gyrozd,50);

%%smoothing function that doesn't really work that well
%[freq,PSD2] = smoothPSD(freq, PSD2, 64); 

figure(1)
loglog(freqx, PSD2x)
hold on
loglog(freqy, PSD2y)
loglog(freqz, PSD2z)
grid on
legend('x','y','z')
xlabel('Frequency(Hz)')
ylabel('Noise Density (deg/sec per sqrt(Hz)')
title('Noise Density')

%% time dependence plots
figure(1)
subplot(3,1,1); plot(time,gyrox); xlabel('time'); ylabel('gyrox rad/s');
title('time dependence plots gyro')
subplot(3,1,2); plot(time,gyroy); xlabel('time'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(time,gyroz); xlabel('time'); ylabel('gyroz rad/s');
figure(2)
subplot(3,1,1); plot(time,mag1x); xlabel('time'); ylabel('mag1x T');
title('time dependence plots mag')
subplot(3,1,2); plot(time,mag1y); xlabel('time'); ylabel('mag1y T');
subplot(3,1,3); plot(time,mag1z); xlabel('time'); ylabel('mag1z T');

%% temp dependence plots
figure(3)
subplot(3,1,1); plot(temp,gyrox); xlabel('temp'); ylabel('gyrox rad/s');
title('temp dependence plots gyro using uncalibrated satellite recorded temp')
subplot(3,1,2); plot(temp,gyroy); xlabel('temp'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(temp,gyroz); xlabel('temp'); ylabel('gyroz rad/s');
figure(4)
subplot(3,1,1); plot(temp,mag1x); xlabel('temp'); ylabel('mag1x T');
title('temp dependence plots mag using uncalibrated satellite recorded temp')
subplot(3,1,2); plot(temp,mag1y); xlabel('temp'); ylabel('mag1y T');
subplot(3,1,3); plot(temp,mag1z); xlabel('temp'); ylabel('mag1z T');

%% time vs. temperature
figure(1)
plot(time,temp); xlabel('time (ms)'); ylabel('temp (uncalibrated)');
title('uncalibrated temperature vs time')

function [freq,PSD,RMS,RND] = ProcessNoise(RawData,fs,freqBand)
    % Performs noise computations on given data set.
    % input: vector of data, sample rate, optional(Two element array of frequency band for RND...
    %       calculation. Defaults to [10,40] for 10Hz - 40Hz band
    % output: Frequency Vector (Hz), Vector of PSD output [units^2/rtHz],...
    %     Single value for overall RMS noise [units rms], Single value for overall RND [units/rtHz]
    % A.Cantwell, Analog Devices, 4/29/2012 

    if(~exist('freqBand'))
        freqBand(1) = 10;
        freqBand(2) = 40;
    elseif(freqBand(1)>=freqBand(2))
        error('Lower frequency bound must be less than upper bound')
    end
    RawData = RawData - mean(RawData); %Remove mean from dataset
    RMS = std(RawData); %Computes noise
    %Performs PSD computation using PWELCH method.  Output is in [units]^2/Hz
    [PSD,freq] = pwelch(RawData,[],[],[],fs,'onesided');
    %Average noise on desired interval
    RND = sqrt(mean(PSD(freq>freqBand(1) & freq<freqBand(2))));
    %Converts data from [units]^2/Hz to [units]/rtHz.
    PSD = PSD.^.5;
end

function [freq, PSD] = smoothPSD(freq_IN,PSD_IN,binsize)
    % Performs noise computations on given data set.
    % A.Cantwell, Analog Devices, 4/29/2012
    freq = freq_IN(1:binsize:end-1);
    PSD=sqrt(mean(reshape(PSD_IN(1:floor(length(PSD_IN)/binsize)*binsize).^2,binsize,floor(length(PSD_IN)/binsize))));
end
