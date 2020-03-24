clc
clear all
close all
%%%%%% looks at temperature dependence for gyro and mag from ETU TVC tests
%%%%%% Cold TVC Test on 3/7/2020
%%%%%% last editted Sruti Vutukury 3/23/2020

data = csvread('TVCtest3_7_20.csv',1,0); %%%starts reading from row 2 %continuous temperature steps; need to calibrate
data2 = csvread('ETUColdTest3_7_20.csv',1,1); %%%starts reading from row 2 %discrete temperature steps

%remove 2 random spikes in gyrox data
idx = find(data(:,5) <= -0.08) ; % from first column get values greater then 0.5
data(idx,:) = [] ; % removes the rows idx from data

time = data(:,1);
mag1x = data(:,2);
mag1y = data(:,3);
mag1z = data(:,4); 
%gyro data multiplied by 0.5 to correct for an error in driver script. driver script has since been corrected 
gyrox = data(:,5).*0.5; %set with outliers; removed earlier
gyroy = data(:,6).*0.5;
gyroz = data(:,7).*0.5;
temp =  data(:,8); %satellite recorded temp (not discrete)

%%%discrete temp recordings
temp2 = ones(length(temp),1); %hand recorded temp (discrete)
F2 = data2(1:34,1);
F3 = data2(1:34,2);
F4 = data2(1:34,3);
C1 = data2(1:34,4);

%to correlate hand recorded temp with sat data
div = ceil((length(time)/34)); 
for i = 1:34
    temp2((i-1)*div+1:i*div)= C1(i);
end

%% gyroscope bias instability
Fs = 50; %sampling frequency 50 hz
T  = 1/Fs; %sampling time
tts = time(length(time))-time(1)/1000; %total time in seconds
tth = time(length(time))-time(1)/(1000*3600); %total time in hours
nbins = (time(length(time))-time(1))/20; %number of bins

%convert to readings to deg/sec
gyroxd = gyrox.*(180/pi); 
gyroyd = gyroy.*(180/pi);
gyrozd = gyroz.*(180/pi);

[avarx,taux] = allanvar(gyroxd); adevx = sqrt(avarx);
[avary,tauy] = allanvar(gyroyd); adevy = sqrt(avary);
[avarz,tauz] = allanvar(gyrozd); adevz = sqrt(avarz);

%bias stability in deg/s
bias_stabx = min(adevx); 
bias_staby = min(adevy);
bias_stabz = min(adevz); 

%angle random walk in deg/s^(3/2)
angle_rand_walkx = interp1(taux,adevx,1,'spline'); 
angle_rand_walky = interp1(tauy,adevy,1,'spline');
angle_rand_walkz = interp1(tauz,adevz,1,'spline');

%Angular rate typical zero-rate level change vs. temperature 
%AKA the slope of stdev vs temp plot
%uses the discrete time intervals
stdevs = ones(34,3); %stores standard deviation of gyro data at each temperature interval
div = ceil((length(time)/34)); 
for i = 1:34 %loops over discrete time intervals
    if i == 34
        stdevs(i,1) = std(gyroxd(((i-1)*div+1:length(gyrox))));
        stdevs(i,2) = std(gyroyd(((i-1)*div+1:length(gyroy))));
        stdevs(i,3) = std(gyrozd(((i-1)*div+1:length(gyroz))));
    else
        stdevs(i,1) = std(gyroxd(((i-1)*div+1:i*div)));
        stdevs(i,2) = std(gyroyd(((i-1)*div+1:i*div)));
        stdevs(i,3) = std(gyrozd(((i-1)*div+1:i*div)));
    end
end
figure(1)
subplot(3,1,1); plot(C1,stdevs(:,1)); xlabel('temp [C]'); ylabel('gyrox stdev deg/s');
subplot(3,1,2); plot(C1,stdevs(:,2)); xlabel('temp [C]'); ylabel('gyroy stdev deg/s');
subplot(3,1,3); plot(C1,stdevs(:,3)); xlabel('temp [C]'); ylabel('gyroz stdev deg/s');

%get the slopes in dps/C
fx = gradient(stdevs(:,1)); maxslopex = max(fx); avgslopex = mean(fx);
fy = gradient(stdevs(:,2)); maxslopey = max(fy); avgslopey = mean(fy);
fz = gradient(stdevs(:,3)); maxslopez = max(fz); avgslopez = mean(fz);

figure(2)
loglog(taux,adevx)
hold on
loglog(tauy,adevy)
loglog(tauz,adevz)
xline(1);
xlabel('tau [s]'); ylabel('sigma(tau) [deg/s]')
title('Allan Variance')
legend('gyrox','gyroy','gyroz')
grid on

%% Gyro Noise Density
close all
[freqx,PSD2x,RMSx,RNDx] = ProcessNoise(gyroxd,50); 
%[freq,PSD2] = smoothPSD(freq, PSD2, 64);
[freqy,PSD2y,RMSy,RNDy] = ProcessNoise(gyroyd,50);
[freqz,PSD2z,RMSz,RNDz] = ProcessNoise(gyrozd,50);

loglog(freqx, PSD2x)
hold on
loglog(freqy, PSD2y)
loglog(freqz, PSD2z)
grid on
legend('x','y','z')
xlabel('Frequency(Hz)')
ylabel('Noise Density (deg/sec per sqrt(Hz)')
title('Noise Density')

%% get basic stats about gyro/mag
a = [15370, 320400, 625500,930500,1235000,1540000,1845000,2150000,2456000,2760000,3065000,...
    3371000,3676000,3981000,4285000,4590000,5200000,5505000,5810000,6115000,6420000,...
    6725000,7030000,7335000];
b = [20220,325200,630300,935300,1240000,1545000,1850000,2155000,2460000,2765000,...
    3070000,3375000,3680000,3985000,4290000,4595000,5205000,5510000,5815000,6120000,...
    6425000,6730000,7035000,7340000];
diffs = ones(24,6);
means = ones(24,6);
stdevs = ones(24,6);
RMSvals = ones(24,6);
avgtemp = ones(24,1);
for i = 1:24
    [~,idx2] = (min(abs(time(:) - a(i))));
    [~,idx3] = (min(abs(time(:) - b(i))));
    avgtemp(i) = mean(temp(idx2:idx3));
    for j = 1:6
        if j == 1
            diffs(i,j) = max(mag1x(idx2:idx3)) - min(mag1x(idx2:idx3));
            means(i,j) = mean(mag1x(idx2:idx3));
            stdevs(i,j) = std(mag1x(idx2:idx3));
            RMSvals(i,j) = rms(mag1x(idx2:idx3));
        elseif j == 2
            diffs(i,j) = max(mag1y(idx2:idx3)) - min(mag1y(idx2:idx3));
            means(i,j) = mean(mag1y(idx2:idx3));
            stdevs(i,j) = std(mag1y(idx2:idx3));
            RMSvals(i,j) = rms(mag1y(idx2:idx3));
        elseif j == 3
            diffs(i,j) = max(mag1z(idx2:idx3)) - min(mag1z(idx2:idx3));
            means(i,j) = mean(mag1z(idx2:idx3));
            stdevs(i,j) = std(mag1z(idx2:idx3));
            RMSvals(i,j) = rms(mag1z(idx2:idx3));
        elseif j == 4
            diffs(i,j) = max(gyrox(idx2:idx3)) - min(gyrox(idx2:idx3));
            means(i,j) = mean(gyrox(idx2:idx3));
            stdevs(i,j) = std(gyrox(idx2:idx3));
            RMSvals(i,j) = rms(gyrox(idx2:idx3));
        elseif j == 5
            diffs(i,j) = max(gyroy(idx2:idx3)) - min(gyroy(idx2:idx3));
            means(i,j) = mean(gyroy(idx2:idx3));
            stdevs(i,j) = std(gyroy(idx2:idx3));
            RMSvals(i,j) = rms(gyroy(idx2:idx3));
        elseif j == 6
            diffs(i,j) = max(gyroz(idx2:idx3)) - min(gyroz(idx2:idx3));
            means(i,j) = mean(gyroz(idx2:idx3));
            stdevs(i,j) = std(gyroz(idx2:idx3));
            RMSvals(i,j) = rms(gyroz(idx2:idx3));
        end
    end
end

%% time dependence plots
figure(1)
subplot(3,1,1); plot(time,gyrox); xlabel('time'); ylabel('gyrox rad/s');
subplot(3,1,2); plot(time,gyroy); xlabel('time'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(time,gyroz); xlabel('time'); ylabel('gyroz rad/s');
figure(2)
subplot(3,1,1); plot(time,mag1x); xlabel('time'); ylabel('mag1x T');
subplot(3,1,2); plot(time,mag1y); xlabel('time'); ylabel('mag1y T');
subplot(3,1,3); plot(time,mag1z); xlabel('time'); ylabel('mag1z T');

%% temp dependence plots (continuous temp recorded by satellite)
figure(3)
subplot(3,1,1); plot(temp,gyrox); xlabel('temp'); ylabel('gyrox rad/s');
subplot(3,1,2); plot(temp,gyroy); xlabel('temp'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(temp,gyroz); xlabel('temp'); ylabel('gyroz rad/s');
figure(4)
subplot(3,1,1); plot(temp,mag1x); xlabel('temp'); ylabel('mag1x T');
subplot(3,1,2); plot(temp,mag1y); xlabel('temp'); ylabel('mag1y T');
subplot(3,1,3); plot(temp,mag1z); xlabel('temp'); ylabel('mag1z T');

%% temp dependence plots (discrete temp recorded by satellite)
% % figure(3)
% % subplot(3,1,1); plot(temp2(1:length(temp)),gyrox); xlabel('temp'); ylabel('gyrox rad/s');
% % subplot(3,1,2); plot(temp2(1:length(temp)),gyroy); xlabel('temp'); ylabel('gyroy rad/s');
% % subplot(3,1,3); plot(temp2(1:length(temp)),gyroz); xlabel('temp'); ylabel('gyroz rad/s');
% % figure(3)
% % subplot(3,1,1); plot(temp2(1:length(temp)),mag1x); xlabel('temp'); ylabel('mag1x T');
% % subplot(3,1,2); plot(temp2(1:length(temp)),mag1y); xlabel('temp'); ylabel('mag1y T');
% % subplot(3,1,3); plot(temp2(1:length(temp)),mag1z); xlabel('temp'); ylabel('mag1z T');

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
