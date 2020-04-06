%if Follower
%voltages_measuredF = {};
%inclinometerPlotsF = {};

% %face2 photodiodes 1:4
% [voltages_measuredF{1}, voltages_measuredF{2}, voltages_measuredF{3}, voltages_measuredF{4}, inclinometerPlotsF{2}] = get_log('Face2+XFollower10011528.log');
% %face3 photodiodes 5:8
% [voltages_measuredF{5}, voltages_measuredF{6}, voltages_measuredF{7}, voltages_measuredF{8}, inclinometerPlotsF{3}] = get_log('Face3-YFollower09205506.log');
% %face4 photodiodes 9:12
% [voltages_measuredF{9}, voltages_measuredF{10}, voltages_measuredF{11}, voltages_measuredF{12}, inclinometerPlotsF{4}] = get_log('newFace4-XFollower11220604.log');
% %face5 photodiodes 13:16
% [voltages_measuredF{13}, voltages_measuredF{14}, voltages_measuredF{15}, voltages_measuredF{16}, inclinometerPlotsF{5}] = get_log('Face3-YLeader16204111.log');
% %face6 photodiodes 17:20
% [voltages_measuredF{17}, voltages_measuredF{18}, voltages_measuredF{19}, voltages_measuredF{20}, inclinometerPlotsF{6}] = get_log('Face6+ZFollower11011312.log');        


%if Leader
voltages_measuredL = {};
inclinometerPlotsL = {};

%face2 photodiodes 1:4
[voltages_measuredL{1}, voltages_measuredL{2}, voltages_measuredL{3}, voltages_measuredL{4}, inclinometerPlotsL{2}] = get_log('Face2+XLeader13180055.log');
%face3 photodiodes 5:8
[voltages_measuredL{5}, voltages_measuredL{6}, voltages_measuredL{7}, voltages_measuredL{8}, inclinometerPlotsL{3}] = get_log('Face3-YLeader16204111.log');
%face4 photodiodes 9:12
[voltages_measuredL{9}, voltages_measuredL{10}, voltages_measuredL{11}, voltages_measuredL{12}, inclinometerPlotsL{4}] = get_log('Face4-XLeader16214521.log');
%face5 photodiodes 13:16
[voltages_measuredL{13}, voltages_measuredL{14}, voltages_measuredL{15}, voltages_measuredL{16}, inclinometerPlotsL{5}] = get_log('Face5+YLeader16213139.log');
%face6 photodiodes 17:20
[voltages_measuredL{17}, voltages_measuredL{18}, voltages_measuredL{19}, voltages_measuredL{20}, inclinometerPlotsL{6}] = get_log('Face6+ZLeader13212220.log');

   
  function [V_Photo0_trim, V_Photo1_trim, V_Photo2_trim, V_Photo3_trim, inclinometerPlot] = get_log(input)
    %read Josh's sun sensor logs to get voltage-angle relations
    A = {};
    file = input;
    
    %open voltage log data for a single face
    fid = fopen(file,'r');
    tline = fgetl(fid);
    
    while ischar(tline)
        A{length(A)+1} = tline;
        tline = fgetl(fid);
    end
    fclose(fid);
    for k = 1:length(A)-1
        j = 1;
        while A{k}(j) ~= '='
            j = j+1;
        end
        A{k} = A{k}(j+1:end);
    end
    
    inclinometerX = [];
    inclinometerY = [];
    V_Photo0 = [];
    V_Photo1 = [];
    V_Photo2 = [];
    V_Photo3 = [];
    for l = 1:6:length(A)
        inclinometerX(length(inclinometerX)+1) = str2double(A{l});
    end
    for l = 2:6:length(A)
        inclinometerY(length(inclinometerY)+1) = str2double(A{l});
    end
    for m = 3:6:length(A)
        V_Photo0(length(V_Photo0)+1) = str2double(A{m});
    end
    for m = 4:6:length(A)
        V_Photo1(length(V_Photo1)+1) = str2double(A{m});
    end
    for m = 5:6:length(A)
        V_Photo2(length(V_Photo2)+1) = str2double(A{m});
    end
    for m = 6:6:length(A)
        V_Photo3(length(V_Photo3)+1) = str2double(A{m});
    end
    
    %select data range
    figure
    plot(V_Photo0)
    title('Click on Left and Right to Choose X Bounds')
    [x,y] = ginput(2);
    x = sort(x);
    close
    x = floor(x);
    
    % trim data to range
    inclinometerX_trim = inclinometerX(x(1):x(2));
    inclinometerY_trim = inclinometerY(x(1):x(2));
    V_Photo0_trim = V_Photo0(x(1):x(2));
    V_Photo1_trim = V_Photo1(x(1):x(2));
    V_Photo2_trim = V_Photo2(x(1):x(2));
    V_Photo3_trim = V_Photo3(x(1):x(2));
    
    % apply low pass filter
    V_Photo0_filt = V_Photo0_trim;
    V_Photo1_filt = V_Photo1_trim;
    V_Photo2_filt = V_Photo2_trim;
    V_Photo3_filt = V_Photo3_trim;
    
    inclinometerX_filt = inclinometerX_trim;
    inclinometerY_filt = inclinometerY_trim;
    a = 0.92;
    for i = 2:length(V_Photo0_filt)
        V_Photo0_filt(i) = V_Photo0_filt(i-1)*a + V_Photo0_filt(i)*(1-a);
        V_Photo1_filt(i) = V_Photo1_filt(i-1)*a + V_Photo1_filt(i)*(1-a);
        V_Photo2_filt(i) = V_Photo2_filt(i-1)*a + V_Photo2_filt(i)*(1-a);
        V_Photo3_filt(i) = V_Photo3_filt(i-1)*a + V_Photo3_filt(i)*(1-a);
        inclinometerY_filt(i) = inclinometerY_filt(i-1)*a + inclinometerY_filt(i)*(1-a);
        inclinometerX_filt(i) = inclinometerX_filt(i-1)*a + inclinometerX_filt(i)*(1-a);
    end    
    
    %get angle plot (y-axis angle)
    if file(1) == 'B'
        inclinometerPlot = inclinometerX_trim;
    else
        inclinometerPlot = inclinometerY_trim;
    end

end
