close all
clear all
    %% read data
    A = {};
    [file, path] = uigetfile({'*.*'});
    fid = fopen([path file],'r');
    tline = fgetl(fid);
    mkdir([path 'pics'],file(1:end-4));
    
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
   
    %% select data range
    figure
    plot(V_Photo0)
    title('Click on Left and Right to Choose X Bounds')
    [x,y] = ginput(2);
    x = sort(x);
    close
    x = floor(x);
    
    %% trim data to range
    inclinometerX_trim = inclinometerX(x(1):x(2));
    inclinometerY_trim = inclinometerY(x(1):x(2));
    V_Photo0_trim = V_Photo0(x(1):x(2));
    V_Photo1_trim = V_Photo1(x(1):x(2));
    V_Photo2_trim = V_Photo2(x(1):x(2));
    V_Photo3_trim = V_Photo3(x(1):x(2));
    
    %% apply low pass filter
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
    
    %% plot V_Photo0
    figure
    plot(V_Photo0_trim)
    title('V_Photo0')
    ylabel('Voltage')
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo0_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo0_' file(1:end-4) '.fig']);
    
     %% plot V_Photo1
    figure
    plot(V_Photo1_trim)
    title('V_Photo1')
    ylabel('Voltage')
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo1_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo1_' file(1:end-4) '.fig']);
    
     %% plot V_Photo2
    figure
    plot(V_Photo2_trim)
    title('V_Photo2')
    ylabel('Voltage')
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo2_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo2_' file(1:end-4) '.fig']);
    
     %% plot V_Photo3
    figure
    plot(V_Photo3_trim)
    title('V_Photo3')
    ylabel('Voltage')
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo_3' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\V_Photo_3' file(1:end-4) '.fig']);
    
    %% Plot Angle
    if file(1) == 'B'
        inclinometerPlot = inclinometerX_trim;
    else
        inclinometerPlot = inclinometerY_trim;
    end
    figure
    plot(inclinometerPlot)
    title('Angle')
    ylabel('Angle')
    saveas(gcf,[path 'pics\' file(1:end-4) '\inclinometer_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\inclinometer_' file(1:end-4) '.fig']);
    
     %% Plot Angle Dependence Data
    for i = 0:3
    figure
    hold on 
    eval(['scatter(inclinometerPlot,V_Photo' num2str(i) '_trim,''k.'')']);
        
    title(['Voltage' num2str(i) ' vs. Angle'])
    
    xlabel('Angle')
    ylabel('Voltage')
    hold off
    eval(['saveas(gcf,[path ''pics\'' file(1:end-4) ''\scatter'' num2str(i) ''_'' file(1:end-4) ''.jpg''])']);
    hold off
    end
    %% Scatter All Data
    figure
    scale = max([V_Photo0_trim V_Photo1_trim V_Photo2_trim V_Photo3_trim]);
    plot((V_Photo0_trim - min(V_Photo0_trim))/scale*2 - 1)
    hold on
    plot((V_Photo1_trim - min(V_Photo1_trim))/scale*2 - 1)
    plot((V_Photo2_trim - min(V_Photo2_trim))/scale*2 - 1)
    plot((V_Photo3_trim - min(V_Photo3_trim))/scale*2 - 1)
    
    plot((inclinometerPlot - min(inclinometerPlot))/max(inclinometerPlot)*2 - 1)
    hold off
    legend({'Voltage0','Voltage1','Voltage2','Voltage3','Angle'})
    saveas(gcf,[path 'pics\' file(1:end-4) '\all_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\all_' file(1:end-4) '.fig']);

   
    
    
     %% Plot Angle Dependence Data
    figure
     hold on 
    scatter(inclinometerPlot,V_Photo0_trim,'r.');
    scatter(inclinometerPlot,V_Photo1_trim,'g.');
    scatter(inclinometerPlot,V_Photo2_trim,'b.');
    scatter(inclinometerPlot,V_Photo3_trim,'k.');
        
    title(['Voltage vs. Angle'])
    
    xlabel('Angle')
    ylabel('Voltage')
    
    
    hold off
    
    
    %% Overlay Low-Pass Filter
    hold on
    plot(inclinometerPlot,V_Photo0_filt,'g')
    plot(inclinometerPlot,V_Photo1_filt,'b')
    plot(inclinometerPlot,V_Photo2_filt,'k')
    plot(inclinometerPlot,V_Photo3_filt,'r')
    
    hold off
    saveas(gcf,[path 'pics\' file(1:end-4) '\scatter_' file(1:end-4) '.jpg']);
    saveas(gcf,[path 'pics\' file(1:end-4) '\scatter_' file(1:end-4) '.fig']);
    
    
    %% write to csv
    M = [inclinometerX_trim' inclinometerY_trim' V_Photo0_trim' V_Photo1_trim' V_Photo2_trim' V_Photo3_trim'];
    csvwrite([path file(1:end-4) 'raw_Data.csv'],M);
    
    %%
%     %% Overlay Both Graphs
%     figure
%     plot((V_Photo_trim - min(V_Photo_trim))/max(V_Photo_trim)*2 - 1)
%     hold on
%     plot((inclinometer_trim - min(inclinometer_trim))/max(inclinometer_trim)*2 - 1)
%     hold off
%     legend({'Voltage','Angle'})
%     saveas(gcf,[path 'pics\' file(1:end-4) '\both_' file(1:end-4) '.jpg']);
%     saveas(gcf,[path 'pics\' file(1:end-4) '\both_' file(1:end-4) '.fig']);
%     
%     %% Plot Angle Dependence Data
%     figure
%     scatter(inclinometer_trim,V_Photo_trim,'k.')
%     title('Voltage vs. Angle')
%     xlabel('Angle')
%     ylabel('Voltage')
%     hold on 
%         % Overlay Low-Pass Filter
%     plot(inclinometer_filt,V_Photo_filt,'r')
%     hold off
%     saveas(gcf,[path 'pics\' file(1:end-4) '\scatter_' file(1:end-4) '.jpg']);
%     saveas(gcf,[path 'pics\' file(1:end-4) '\scatter_' file(1:end-4) '.fig']);
%     
%     %% Plot Graph on Polar Coordinates
%     figure
%     polarscatter((inclinometer_trim)/180*pi,V_Photo_trim,'b.')
%     title('Polar Plot')
%     saveas(gcf,[path 'pics\' file(1:end-4) '\polar_' file(1:end-4) '.jpg']);
%     saveas(gcf,[path 'pics\' file(1:end-4) '\polar_' file(1:end-4) '.fig']);
%     
%     %% Save Workspace
%     save([path 'pics\' file(1:end-4) '\workspace_' file(1:end-4) '.mat'])
%     
%     %% animation
%     figure
%     hold on
%     pause(2)
%     time = 20;
%     for i = 1:5:length(V_Photo_trim)
%         scatter(inclinometer_trim(i:i+5),V_Photo_trim(i:i+5),'k.')
%         pause(.0001)
%     end
%     