data = csvread('TVCtest3_7_20.csv',1,0); %%%starts reading from row 2
data2 = csvread('ETUColdTest3_7_20.csv',1,1); %%%starts reading from row 2

%remove 2 random spikes in gyrox data
idx = find(data(:,5) <= -0.08) ; % from first column get values greater then 0.5
data(idx,:) = [] ; % removes the rows idx from data

time = data(:,1);
mag1x = data(:,2);
mag1y = data(:,3);
mag1z = data(:,4); 
gyrox = data(:,5); %set with outliers; removed earlier
gyroy = data(:,6);
gyroz = data(:,7);
temp =  data(:,8); %sat recorded temp (not discrete)

%%%discrete temp recordings
% temp2 = ones(length(temp),1); %hand recorded temp (discrete)
% F2 = data2(1:34,1);
% F3 = data2(1:34,2);
% F4 = data2(1:34,3);
% C1 = data2(1:34,4);
% %to correlate hand recorded temp with sat data
% div = ceil((length(time)/34)); 
% for i = 1:34
%     temp2((i-1)*div+1:i*div)= C1(i);
% end

%%% get stats about mag
% idx2 = find(time(:) == 320413)
% idx3 = find(time(:) == 325263)
% 
% diff = max(gyrox(254:496)) - min(gyrox(254:496))
% mean = mean(gyrox(254:496))
% median = median(gyrox(254:496))
% std = std(gyrox(254:496))

% time dependence plots
figure(1)
subplot(3,1,1); plot(time,gyrox); xlabel('time'); ylabel('gyrox rad/s');
subplot(3,1,2); plot(time,gyroy); xlabel('time'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(time,gyroz); xlabel('time'); ylabel('gyroz rad/s');

figure(2)
subplot(3,1,1); plot(time,mag1x); xlabel('time'); ylabel('mag1x T');
subplot(3,1,2); plot(time,mag1y); xlabel('time'); ylabel('mag1y T');
subplot(3,1,3); plot(time,mag1z); xlabel('time'); ylabel('mag1z T');

% temp dependence plots
figure(3)
% subplot(3,1,1); plot(temp2(1:length(temp)),gyrox); xlabel('temp'); ylabel('gyrox rad/s');
% subplot(3,1,2); plot(temp2(1:length(temp)),gyroy); xlabel('temp'); ylabel('gyroy rad/s');
% subplot(3,1,3); plot(temp2(1:length(temp)),gyroz); xlabel('temp'); ylabel('gyroz rad/s');
subplot(3,1,1); plot(temp,gyrox); xlabel('temp'); ylabel('gyrox rad/s');
subplot(3,1,2); plot(temp,gyroy); xlabel('temp'); ylabel('gyroy rad/s');
subplot(3,1,3); plot(temp,gyroz); xlabel('temp'); ylabel('gyroz rad/s');

figure(4)
% subplot(3,1,1); plot(temp2(1:length(temp)),mag1x); xlabel('temp'); ylabel('mag1x T');
% subplot(3,1,2); plot(temp2(1:length(temp)),mag1y); xlabel('temp'); ylabel('mag1y T');
% subplot(3,1,3); plot(temp2(1:length(temp)),mag1z); xlabel('temp'); ylabel('mag1z T');
subplot(3,1,1); plot(temp,mag1x); xlabel('temp'); ylabel('mag1x T');
subplot(3,1,2); plot(temp,mag1y); xlabel('temp'); ylabel('mag1y T');
subplot(3,1,3); plot(temp,mag1z); xlabel('temp'); ylabel('mag1z T');

%% checking average time between each sample taken
timeInt = zeros(length(data)-1,1);
for d = 1:length(timeInt)
    val = data(d+1,1) - data(d,1);
    timeInt(d) = val;
end
%diff = rmoutliers(timeInt, 'mean');