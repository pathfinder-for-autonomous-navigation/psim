function helperFrequencyAnalysisPlot1(F,Ymag,Yangle,NFFT,ttlMag,label,ttlPhase)
% Plot helper function for the FrequencyAnalysisExample
% Copyright 2012 The MathWorks, Inc.

figure(1)
subplot(3,1,1)
plot(F(1:NFFT/2),20*log10(Ymag(1:NFFT/2)));
title('Magnitude response of the mag signal waveform')
xlabel('Frequency Hz'); ylabel('T')
grid on; axis tight 

subplot(3,1,2)
plot(F(1:NFFT/2),Yangle(1:NFFT/2));
title('Phase response of the mag signal waveform')
xlabel('Frequency in Hz'); ylabel('radians')
grid on; axis tight
end