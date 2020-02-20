clc
clear all
close all
tic 

data = csvread('total.csv',1,0); %%%starts reading from row 2
data(:,8) = data(:,8)/(2*pi); %wheel 0 on [Hz]
data(:,9) = data(:,9)/(2*pi);
data(:,10) = data(:,10)/(2*pi);

data(:,19) = data(:,19)/(2*pi); 
data(:,20) = data(:,20)/(2*pi);%wheel 1 on [Hz]
data(:,21) = data(:,21)/(2*pi);

data(:,30) = data(:,30)/(2*pi); 
data(:,31) = data(:,31)/(2*pi);
data(:,32) = data(:,32)/(2*pi);%wheel 2 on [Hz]

diff = zeros(length(data)-1,1);
for d = 1:length(diff)
    val = data(d+1,1) - data(d,1);
    diff(d) = val;
end
diff = rmoutliers(diff, 'mean');
mean(diff);
std(diff);
var(diff);
    
%figure(1)
%plot(data(:,1),data(:,5))
%%%% data sheet indexing
%%% 1  2  3  4  5  6  7     8     9     10    11 
%%% t1 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0   %wheel 0 on
%%% 12 13 14 15 16 17 18   19     20     21   22
%%% t2 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0  %wheel 1 on
%%% 23 24 25 26 27 28 29   30     31     32
%%% t3 1x 1y 1z 2x 2y 2z speed0 speed1 speed2 0 %wheel 2 on

access = [1 2 8; 1 3 8; 1 4 8; 1 5 8; 1 6 8; 1 7 8;....
    12 13 20; 12 14 20; 12 15 20; 12 16 20; 12 17 20; 12 18 20; ...
    23 24 32; 23 25 32; 23 26 32; 23 27 32; 23 28 32; 23 29 32];
%%% mag [1/2][x/y/z][wheel 0/1/2 on]
% names = {'mag1x0';'mag1y0';'mag1z0';'mag2x0';'mag2y0';'mag2z0';
%     'mag1x1';'mag1y1';'mag1z1';'mag2x1';'mag2y1';'mag2z1';
%     'mag1x2';'mag1y2';'mag1z2';'mag2x2';'mag2y2';'mag2z2'};

for s = 1:1 %loops of 18 data sets that each consist of full sweep
    
    start = data(access(s,1),1);
    freq = ones(10,1); freq(1) = data(1,access(s,3)); %take wheel reading
    idx = ones(10,1);
    step = ceil(length(data)/10);
    %%% get indices for each frequency step start in data set
    for i = 1:11
        %time = start + 10000*(i-1); %each frequency step is for 10 sec
        %x = find(data(:,access(s,1)) > time, 1,'first');
        %idx(i) = x;
        idx(i) = (step*i)/2; %middle of the data
    end
    idx1 = [0,520,1040,1560,2080,2600,3120,3640,4160,4680,5200]';
    %idx = [260,780,1300,1820,2340,2860,3380,3900,4420,4940];
    idx = [322,780,1362,1802,2301,2913,3349,3837,4414,4802]';
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
    peaks = [];
    for j = 2:length(idx1)
        %NFFT = length(data(idx(j-1):idx(j),access(s,2))); %take mag reading can be 1/2 x/y/z
        NFFT = length(data(idx1(j-1)+1:idx1(j),access(s,2)));
        Y = fft(data(idx1(j-1)+1:idx1(j),access(s,2)), NFFT); 
        F = ((0:1/NFFT:1-1/NFFT)*F_samp).';
        magY = abs(Y); %magnitude of the fft
        
        phaseY = unwrap(angle(Y));
        
%         NFFTs(j) = NFFT; Fs{j} = F;
%         Ys{j} = Y; magnitudeYs{j} = magnitudeY;
%         phaseYs{j} = phaseY;
        
        %magY=mag2db(magY);
        subplot(2,1,1);plot(F(1:NFFT/2),magY(1:NFFT/2));
        title('Magnitude response of signal');
        xlabel('Frequency Hz')
        ylabel('Magnitude(dB)');
        
        subplot(2,1,2);plot(F(1:NFFT/2),phaseY(1:NFFT/2));
        title('Phase response of signal');
        xlabel('Frequency Hz')
        ylabel('radians');

        fname = num2str(j);
        save(fname)
        saveas(gcf,fname)
        
        fprintf('finished plotting freq %f \n',j);
        peaks = [peaks max(magY)];
    end
    figure(2)
    plot(peaks(1:3),freq(1:3))
    diff = [6.346,12.02,18.75,24.42];
    figure(3)
    plot(diff)
    
    %%% plot full frequency sweep mag reading vs. frequency of wheel;
    %%% using fft for each individual frequency step 
    %[curvefit,gof,output] = fit(F(1:NFFT/2),20*log10(magnitudeY(1:NFFT/2)),'smoothingspline','normalize','on');
%     figure(1)
%     hold on
%     n = length(idx);
%     P = {}; %cell of information for each frequency step
%     for k = 2:length(idx)
%         x_array = Fs{k};
%         y_array = magnitudeYs{k};
%         y_array = rmoutliers(y_array,'mean'); %remove pts. more than 3 stdevs from the mean
%         [curvefit,gof,output] = fit(x_array,y_array,'smoothingspline','normalize','on');
%         plot(curvefit,x_array,y_array);
%         plot(F(1:NFFT/2),20*log10(magnitudeY(1:NFFT/2)));
%         plot(curvefit(1:NFFT/2),F(1:NFFT/2));
%         p.formula = formula(curvefit);
%         p.mean = mean(magnitudeY); p.std = std(magnitudeY); p.var = var(magnitudeY);
%         p.max = max(magnitudeY);  p.min = min(magnitudeY);
%         c = coeffvalues(curvefit); c = c.coefs; p.coeffs = c;
%         P{k} = p; %packet of information for each frequency step
%         methods(curvefit)
%         fprintf('plotted freq %f\n',k);
%     end
%     hold off
%     legend('hide'); grid on;
%     xlabel('wheel frequency Hz'); ylabel('mag reading T'); 
%     fname = char(names{s});
%     save(fname)
%     fname = (names{s});
%     saveas(gcf,fname)
    
    fprintf('finished data set %f\n',s);
    %close all
    %clf
end