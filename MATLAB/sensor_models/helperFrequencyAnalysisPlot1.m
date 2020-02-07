function helperFrequencyAnalysisPlot1(F,Ymag,Yangle,NFFT,ttlMag,label,ttlPhase)
% Plot helper function for the FrequencyAnalysisExample

% Copyright 2012 The MathWorks, Inc.

figure(1)
subplot(3,1,1)
plot(F(1:NFFT/2),20*log10(Ymag(1:NFFT/2)));
if nargin > 4 && ~isempty(ttlMag)
  tstr = {'Magnitude response of the mag signal waveform ',ttlMag};
else
  tstr = {'Magnitude response of the mag signal waveform'};
end
title(tstr)
xlabel('Frequency Hz')
label = 'T';
ylabel(label)
grid on;
axis tight 
subplot(3,1,2)
plot(F(1:NFFT/2),Yangle(1:NFFT/2));
if nargin > 6
  tstr = {'Phase response of the mag signal waveform',ttlPhase};
else  
  tstr = {'Phase response of the mag signal waveform'};
end
title(tstr)
xlabel('Frequency in Hz')
ylabel('radians')
grid on;
axis tight
end