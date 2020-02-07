clc
clear all
close all
data = csvread('total.csv',2,0);

data(:,8) = data(:,8)/(2*pi); %wheel 0 on [Hz]
data(:,9) = data(:,9)/(2*pi);
data(:,10) = data(:,10)/(2*pi);

figure(1)
plot(data(:,1),data(:,5))
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
names = {'mag1x0';'mag1y0';'mag1z0';'mag2x0';'mag2y0';'mag2z0';
    'mag1x1';'mag1y1';'mag1z1';'mag2x1';'mag2y1';'mag2z1';
    'mag1x2';'mag1y2';'mag1z2';'mag2x2';'mag2y2';'mag2z2'};

for s = 1:length(access)
    start = data(access(s,1),1);
    freq = ones(10,1); freq(1) = data(1,access(s,3)); %take wheel reading
    idx = ones(10,1); idx(1) = 1;
    for i = 2:11
        time = start + 10000*(i-1);
        x = find(data(:,access(s,1)) > time, 1,'first');
        idx(i) = x;
    end
    for i = 1:10
        freq(i) = data(idx(i),access(s,3));
    end

    Fs = 50;  %mag sampling freq
    NFFT = length(data(1:idx(2),access(s,2))); %take mag reading can be 1/2 x/y/z
    Y = fft(data(1:idx(2),access(s,2)), NFFT);
    F = ((0:1/NFFT:1-1/NFFT)*Fs).';
    magnitudeY = abs(Y);
    phaseY = unwrap(angle(Y));
    figure(gcf)
    %plot(F,magnitudeY,phaseY,NFFT)
    figure(2)
    helperFrequencyAnalysisPlot1(F,magnitudeY,phaseY,NFFT)
    subplot(3,1,3)
    %[curvefit,gof,output] = fit(F(1:NFFT/2),20*log10(magnitudeY(1:NFFT/2)),'smoothingspline','normalize','on');
    [curvefit,gof,output] = fit(F,20*log(magnitudeY),'smoothingspline')%'normalize','on');
    plot(curvefit,F,20*log(magnitudeY));
    %plot(F(1:NFFT/2),20*log10(magnitudeY(1:NFFT/2)));
    %plot(curvefit(1:NFFT/2),F(1:NFFT/2));
    xlabel('frequency Hz')
    ylabel('mag reading T')
    px.formula = formula(curvefit);
    px.mean = mean(magnitudeY); px.std = std(magnitudeY); px.var = var(magnitudeY);
    px.max = max(magnitudeY);  px.min = min(magnitudeY);
    %methods(curvefit)
    c = coeffvalues(curvefit); c = c.coefs;
    fname = char(names{s});
    save(fname)
    fname = (names{s});
    saveas(gcf,fname)
end

% for i = length(access)
%     start = data(1,1);
%     freq = ones(10,1); freq(1) = data(1,8);
%     idx = ones(10,1); idx(1) = 1;
%     for i = 2:11
%         time = start + 10000*(i-1);
%         x = find(data(:,1) > time, 1,'first');
%         idx(i) = x;
%     end
%     for i = 1:10
%         freq(i) = data(idx(i),8);
%     end
% 
%     Fs = 50;  %mag sampling freq
%     NFFT = length(data(1:idx(2),5));
%     Y = fft(data(1:idx(2),5), NFFT);
%     F = ((0:1/NFFT:1-1/NFFT)*Fs).';
%     magnitudeY = abs(Y);
%     phaseY = unwrap(angle(Y));
%     figure(gcf)
%     %plot(F,magnitudeY,phaseY,NFFT)
%     figure(1)
%     helperFrequencyAnalysisPlot1(F,magnitudeY,phaseY,NFFT)
%     figure(2)
%     [curvefit,gof,output] = fit(F,magnitudeY,'smoothingspline','normalize','on');
%     plot(curvefit,F,magnitudeY);
%     xlabel('frequency Hz')
%     ylabel('mag reading T')
% 
% end
 
% % plot(time0,speedA0)
% % title('time domain'); xlabel('s'); ylabel('T')
% % samp_freq = 50; %sampling frequency
% % L0 = (time0(length(time0)-1)-time0(1))/samp_freq; %signal length
% % n0 = 2^nextpow2(L0);
% % Y0 = fft(speedA0,n0);
% % f0 = samp_freq*(0:(n0/2))/n0;'
% %                          
% % P0 = abs(Y0/n0);
% % subplot(3,2,2)
% % %plot(f0,P0(1:(n0/2)+1));
% % plot(P0, mag1x0);
% % title('freq domain'); xlabel('Hz'); ylabel('abs(P(f))')
% 
% %%%% mag 1 only
% [curvefit1x0,gof1x0,output1x0] = fit(time0,mag1x0,'smoothingspline','normalize','on'); %wheel 0 x
% figure(1)
% subplot(3,3,1)
% plot(curvefit1x0,time0,mag1x0);
% title('wheel 0 on, mag1x reading')

% %%%% driver noise data for 'testwheelnoise.csv' data 
% figure(2)
% [dcurvex,dgofx,doutputx] = fit(time(1:a),mag1x(1:a),'smoothingspline','normalize','on');
% subplot(3,1,1)
% plot(dcurvex,time(1:a),mag1x(1:a));
% title('driver vibes x reading')

%methods(curvefitx)
%coeffxs = coeffvalues(curvefitx); coeffxs = coeffxs.coefs;
% figure(1)
% subplot(3,1,1)
% plot(curvefitx,time,mag1x);
% hold on
% plot(curvefitx,time,mag1x,'Residuals')
% 
% xlabel('time(s)'); ylabel('mag1x (T)')
% %plot(curvefit,time,mag1x,'predfunc')
% formula(curvefitx)
% px.mean = mean(mag1x); px.std = std(mag1x); px.var = var(mag1x);
% px.max = max(mag1x);  px.min = min(mag1x);
% A = time;
% B = mag1x;
% C = [1 0 0];
% D = 0;
% %Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
% %[kalmf,L,~,M,Z] = kalman(Plant,1,1);
% disp(px)
