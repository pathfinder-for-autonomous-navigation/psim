%%%%%% magnetometer characterization using wheel frequency sweeps
%%%%%% wheel ramp up test on 2/13/2020
%%%%%% last editted Sruti Vutukury 3/23/2020

clc
clear all
close all
tic 
warning('off','all') %disabled warning related to large output file sizes

data = csvread('mag_data_2_13_20.csv',1,0); %%%starts reading from row 2

%convert wheel freq from rad/sec to Hz
data(:,8) = data(:,8)/(2*pi); %wheel 0 on [Hz]
data(:,9) = data(:,9)/(2*pi);
data(:,10) = data(:,10)/(2*pi);

data(:,19) = data(:,19)/(2*pi); 
data(:,20) = data(:,20)/(2*pi);%wheel 1 on [Hz]
data(:,21) = data(:,21)/(2*pi);

data(:,30) = data(:,30)/(2*pi); 
data(:,31) = data(:,31)/(2*pi);
data(:,32) = data(:,32)/(2*pi);%wheel 2 on [Hz]

%% check time domain plots
figure(1); plot(data(:,1),data(:,8)); title('wheel0 spin [rad/sec] vs time [ms]'); %wheel 0 on
saveas(gcf,'wheel0tdomainplot'); 

figure(2); plot(data(:,12),data(:,20)); title('wheel1 spin [rad/sec] vs time [ms]'); %wheel 1 on
saveas(gcf,'wheel1tdomainplot'); 

figure(3); plot(data(:,23),data(:,32)); title('wheel2 spin [rad/sec] vs time [ms]'); %wheel 2 on
saveas(gcf,'wheel2tdomainplot');

%% checking average time between each sample taken; roughly 19 ms
timeInt = zeros(length(data)-1,1);
for d = 1:length(timeInt)
    val = data(d+1,1) - data(d,1);
    timeInt(d) = val;
end
diff = rmoutliers(timeInt, 'mean');
mean(timeInt); std(timeInt); var(timeInt);

%% main 
%%%% 'total.csv' data sheet indexing reference
%%% 1  2  3  4  5  6  7     8     9     10    11 
%%% t1 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0   %wheel 0 on
%%% 12 13 14 15 16 17 18   19     20     21   22
%%% t2 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0  %wheel 1 on
%%% 23 24 25 26 27 28 29   30     31     32
%%% t3 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0 %wheel 2 on

%%% access indexing reference
%%% mag [1/2][x/y/z][wheel 0/1/2 on]
access = [1 2 8; 1 3 8; 1 4 8; 1 5 8; 1 6 8; 1 7 8;....
    12 13 20; 12 14 20; 12 15 20; 12 16 20; 12 17 20; 12 18 20; ...
    23 24 32; 23 25 32; 23 26 32; 23 27 32; 23 28 32; 23 29 32];

names = {'mag1x0';'mag1y0';'mag1z0';'mag2x0';'mag2y0';'mag2z0';
    'mag1x1';'mag1y1';'mag1z1';'mag2x1';'mag2y1';'mag2z1';
    'mag1x2';'mag1y2';'mag1z2';'mag2x2';'mag2y2';'mag2z2'};

for s = 1:18 %loops of 18 data sets that each consist of full sweep
    
    start = data(access(s,1),1);
    freq = ones(10,1); freq(1) = data(1,access(s,3)); %take wheel reading
    idx1 = [0,520,1040,1560,2080,2600,3120,3640,4160,4680,5200]'; %start of each set
    %idx = [260,780,1300,1820,2340,2860,3380,3900,4420,4940];
    idx = [322,780,1362,1802,2301,2913,3349,3837,4414,4802]'; %median of set
    
    %get corresponding median frequency
    for i = 1:length(idx)
        freq(i) = data(idx(i),access(s,3));
    end
    
    %%% apply fft to each frequency
    F_samp = 50;  %mag sampling freq
    NFFTs = ones(length(idx),1);
    Ys = {};
    Fs = {};
    magnitudeYs = {};
    phaseYs = {};
    offset = [];
    Ps = {}; %stores stats for each freq
    
    fname3 = [names{s} 'allFFTs'];
    fname2 = [names{s} 'offsets'];
    
    for j = 2:length(idx1) %over entire frequency sweep 
        
        %NFFT = length(data(idx(j-1):idx(j),access(s,2))); %take mag reading can be 1/2 x/y/z
        NFFT = length(data(idx1(j-1)+1:idx1(j),access(s,2))); %length of fft
        Y = fft(data(idx1(j-1)+1:idx1(j),access(s,2)), NFFT); 
        F = ((0:1/NFFT:1-1/NFFT)*F_samp).';
        magY = abs(Y); %magnitude of the fft
        phaseY = unwrap(angle(Y));
        leak = 0; %looking at leakage of signal energy to n bins on either side of a freq; assuming no leak
        
        NFFTs(j) = NFFT; Fs{j} = F;
        Ys{j} = Y; magnitudeYs{j} = magY;
        phaseYs{j} = phaseY;

        fname0 = [names{s} 'freq' num2str(j)];
        fname1 = [names{s} 'freq' num2str(j) '.mat'];
        fname4 = [names{s} 'freq' num2str(j) 'tdomainfit'];
        
        subplot(2,1,1);
        %hold on %uncomment to see all frequencies overlap on eachother
        plot(F(1:NFFT/2),magY(1:NFFT/2));
        title(['Magnitude response of signal: ' fname0]);
        xlabel('Frequency Hz')
        ylabel('Magnitude(T)');
        %legend(fname0,'-DynamicLegend');
        if j == length(idx1)
            hold off
        end
        
        subplot(2,1,2);
        plot(F(1:NFFT/2),phaseY(1:NFFT/2));
        %hold on %uncomment to see all frequencies overlap on eachother
        %gcf
        title(['Phase response of signal: ' fname0]);
        xlabel('Frequency Hz')
        ylabel('radians');
        %legend(fname0,'-DynamicLegend');
        if j == length(idx1)
            hold off
        end
        
        
        %find offset of peaks from 5,10,15....50 Hz
        [M, I] = max(magY(2:NFFT/2));
        peakFreq = F(I);
        offset(j-1) = peakFreq - 5*(j-1);
        
        %get stats of the data in time domain
        x = data(idx1(j-1)+1:idx1(j),access(s,1)); %time array
        y = data(idx1(j-1)+1:idx1(j),access(s,2)); %mag data array
        
        [curvefit,gof,output] = fit(x,y,'smoothingspline','normalize','on');
        plot(curvefit,x,y);
        title(['time domain plot + curvefit: ' fname0])
        
        p.formula = formula(curvefit);
        p.mean = mean(y); p.std = std(y); p.var = var(y);
        p.rms = rms(y);
        p.p2p = max(y)- min(y); %peak to peak of magntitudes
        c = coeffvalues(curvefit); c = c.coefs; p.coeffs = c; 
        %methods(curvefit)
        
        %get signal-to-noise ratio SNR
        [~,m] = max(magY(1:NFFT/2)); %peak of the freq spectrum
        sigs = [m-leak:m+leak NFFT-m-leak:NFFT-m+leak]; %find the bins corresponding to the signal around peak, including leakage
        sig_pow = sum(magY(sigs));
        magY([sigs]) = 0; % making all bins corresponding to signal zero, what remains = noise
        noise_pow = sum(magY);
        p.SNR = 10*log10(sig_pow/noise_pow); %signal-to-noise ratio
        
        Ps{j} = p; %store packet of information for each frequency step
         
        %save(fname1); %uncomment to save fft data .mat file for each frequency
        %saveas(gcf,fname0); %uncomment to save fft plot for each frequency
        %saveas(gcf,fname4); %uncomment to save time domain plot for each frequency
        
        fprintf('finished freq %f \n',j);
    end
   
%    save(fname3); %uncomment to save all FFT data for entire frequency sweep
    
    close all
    x_array = linspace(0,50,10);
    plot(x_array, offset);
    title('offset between mag freq peak and wheel freq');
    xlabel('wheel freq')
    ylabel('offset in Hz');
%     saveas(gcf,fname2); % uncomment to save offset data between mag freq peak and wheel freq
    
    fprintf('finished data set %f\n',s);
    %close all
    %clf
end