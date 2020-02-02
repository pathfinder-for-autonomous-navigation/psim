%mag1 (Lis2mdl),ETU sat, wheels are running
%22C
%M1 = csvread('testwheelnoise.csv',9,0);
data = csvread('total.csv',1,0);
%wheel0 = csvread('wheel0.csv',1,0);
%wheel0 = csvread('wheel1.csv',1,0);
%data = csvread('wheel2.csv',1,0);

%%% wheel 0 on
time0 = data(:,1);
mag1x0 = data(:,2); mag1y0 = data(:,3); mag1z0 = data(:,4);
mag2x0 = data(:,5); mag2y0 = data(:,6); mag2z0 = data(:,7);

%%%wheel 1 on
time1 = data(:,12);
mag1x1 = data(:,13); mag1y1 = data(:,14); mag1z1 = data(:,15);
mag2x1 = data(:,16); mag2y1 = data(:,17); mag2z1 = data(:,18);

%%%% wheel 2 on
time2 = data(:,23);
mag1x2 = data(:,24); mag1y2 = data(:,25); mag1z2 = data(:,26);
mag2x2 = data(:,27); mag2y2 = data(:,28); mag2z2 = data(:,29);

% a = find(data(:,9),~0, 'first'); 
% b = find(data(:,10),~0, 'first'); 
% c = find(data(:,11),~0, 'first');
% d = find(data(:,11),~0, 'last') + 1;

%%%% mag 1 only
[curvefit1x0,gof1x0,output1x0] = fit(time0,mag1x0,'smoothingspline','normalize','on'); %wheel 0 x
[curvefit1y0,gof1y0,output1y0] = fit(time0,mag1y0,'smoothingspline','normalize','on'); %wheel 0 y
[curvefit1z0,gof1z0,output1z0] = fit(time0,mag1z0,'smoothingspline','normalize','on'); %wheel 0 z

[curvefit1x1,gof1x1,output1x1] = fit(time1,mag1x1,'smoothingspline','normalize','on'); %wheel 1 x
[curvefit1y1,gof1y1,output1y1] = fit(time1,mag1y1,'smoothingspline','normalize','on'); %wheel 1 y
[curvefit1z1,gof1z1,output1z1] = fit(time1,mag1z1,'smoothingspline','normalize','on'); %wheel 1 z

[curvefit1x2,gof1x2,output1x2] = fit(time2,mag1x2,'smoothingspline','normalize','on'); %wheel 2 x
[curvefit1y2,gof1y2,output1y2] = fit(time2,mag1y2,'smoothingspline','normalize','on'); %wheel 2 y
[curvefit1z2,gof1z2,output1z2] = fit(time2,mag1z2,'smoothingspline','normalize','on'); %wheel 2 z

%%%% mag 2 only
[curvefit2x0,gof2x0,output2x0] = fit(time0,mag2x0,'smoothingspline','normalize','on'); %wheel 0 x
[curvefit2y0,gof2y0,output2y0] = fit(time0,mag2y0,'smoothingspline','normalize','on'); %wheel 0 y
[curvefit2z0,gof2z0,output2z0] = fit(time0,mag2z0,'smoothingspline','normalize','on'); %wheel 0 z

[curvefit2x1,gof2x1,output2x1] = fit(time1,mag2x1,'smoothingspline','normalize','on'); %wheel 1 x
[curvefit2y1,gof2y1,output2y1] = fit(time1,mag2y1,'smoothingspline','normalize','on'); %wheel 1 y
[curvefit2z1,gof2z1,output2z1] = fit(time1,mag2z1,'smoothingspline','normalize','on'); %wheel 1 z

[curvefit2x2,gof2x2,output2x2] = fit(time2,mag2x2,'smoothingspline','normalize','on'); %wheel 2 x
[curvefit2y2,gof2y2,output2y2] = fit(time2,mag2y2,'smoothingspline','normalize','on'); %wheel 2 y
[curvefit2z2,gof2z2,output2z2] = fit(time2,mag2z2,'smoothingspline','normalize','on'); %wheel 2 z

figure(1)
subplot(3,3,1)
plot(curvefit1x0,time0,mag1x0);
title('wheel 0 on, mag1x reading')
hold on
subplot(3,3,2)
plot(curvefit1y0,time0,mag1y0);
title('wheel 0 on, mag1y reading')
subplot(3,3,3)
plot(curvefit1z0,time0,mag1z0);
title('wheel 0 on, mag1z reading')

subplot(3,3,4)
plot(curvefit1x1,time1,mag1x1);
title('wheel 1 on, mag1x reading')
hold on
subplot(3,3,5)
plot(curvefit1y1,time1,mag1y1);
title('wheel 1 on, mag1y reading')
subplot(3,3,6)
plot(curvefit1z1,time1,mag1z1);
title('wheel 1 on, mag1z reading')

subplot(3,3,7)
plot(curvefit1x2,time2,mag1x2);
title('wheel 2 on, mag1x reading')
hold on
subplot(3,3,8)
plot(curvefit1y2,time2,mag1y2);
title('wheel 2 on, mag1y reading')
subplot(3,3,9)
plot(curvefit1z2,time2,mag1z2);
title('wheel 2 on, mag1z reading')

figure(2)
subplot(3,3,1)
plot(curvefit2x0,time0,mag2x0);
title('wheel 0 on, mag2x reading')
hold on
subplot(3,3,2)
plot(curvefit2y0,time0,mag2y0);
title('wheel 0 on, mag2y reading')
subplot(3,3,3)
plot(curvefit2z0,time0,mag2z0);
title('wheel 0 on, mag2z reading')

subplot(3,3,4)
plot(curvefit2x1,time1,mag2x1);
title('wheel 1 on, mag2x reading')
hold on
subplot(3,3,5)
plot(curvefit2y1,time1,mag2y1);
title('wheel 1 on, mag2y reading')
subplot(3,3,6)
plot(curvefit2z1,time1,mag2z1);
title('wheel 1 on, mag2z reading')

subplot(3,3,7)
plot(curvefit2x2,time2,mag2x2);
title('wheel 2 on, mag2x reading')
hold on
subplot(3,3,8)
plot(curvefit2y2,time2,mag2y2);
title('wheel 2 on, mag2y reading')
subplot(3,3,9)
plot(curvefit2z2,time2,mag2z2);
title('wheel 2 on, mag2z reading')


function plots(time, mag1x, mag1y, mag1z)
%     % x mag reading
%     [curvefitx1,gofx1,outputx1] = fit(time(a:b),mag1x(a:b),'smoothingspline','normalize','on');
%     [curvefitx2,gofx2,outputx2] = fit(time(b:c),mag1x(b:c),'smoothingspline','normalize','on');
%     [curvefitx3,gofx3,outputx3] = fit(time(c:d),mag1x(c:d),'smoothingspline','normalize','on');
% 
%     % y mag reading
%     [curvefity1,gofy1,outputy1] = fit(time(a:b),mag1y(a:b),'smoothingspline','normalize','on');
%     [curvefity2,gofy2,outputy2] = fit(time(b:c),mag1y(b:c),'smoothingspline','normalize','on');
%     [curvefity3,gofy3,outputy3] = fit(time(c:d),mag1y(c:d),'smoothingspline','normalize','on');
% 
%     % z mag reading
%     [curvefitz1,gofz1,outputz1] = fit(time(a:b),mag1z(a:b),'smoothingspline','normalize','on');
%     [curvefitz2,gofz2,outputz2] = fit(time(b:c),mag1z(b:c),'smoothingspline','normalize','on');
%     [curvefitz3,gofz3,outputz3] = fit(time(c:d),mag1z(c:d),'smoothingspline','normalize','on');


    % x mag reading
    [curvefitx1,gofx1,outputx1] = fit(time,mag1x,'smoothingspline','normalize','on');
    [curvefitx2,gofx2,outputx2] = fit(time,mag2x,'smoothingspline','normalize','on');
    
    % y mag reading
    [curvefity1,gofy1,outputy1] = fit(time,mag1y,'smoothingspline','normalize','on');
    [curvefity2,gofy2,outputy2] = fit(time,mag2y,'smoothingspline','normalize','on');

    % z mag reading
    [curvefitz1,gofz1,outputz1] = fit(time,mag1z,'smoothingspline','normalize','on');
    [curvefitz2,gofz2,outputz2] = fit(time,mag2z,'smoothingspline','normalize','on');


    %%% only wheel 1 on
    subplot(3,3,1)
    plot(curvefitx1,time,mag1x);
    title('wheel 1 on, mag1x reading')
    hold on
    subplot(3,3,2)
    plot(curvefity1,time,mag1y);
    title('wheel 1 on, mag1y reading')
    subplot(3,3,3)
    plot(curvefitz1,time,mag1z);
    title('wheel 1 on, mag1z reading')

    %%% only wheel 2 on
    subplot(3,3,4)
    plot(curvefitx2,time(b:c),mag1x(b:c));
    title('wheel 2 on, mag1x reading')
    hold on
    subplot(3,3,5)
    plot(curvefity2,time(b:c),mag1y(b:c));
    title('wheel 2 on, mag1y reading')
    subplot(3,3,6)
    plot(curvefitz2,time(b:c),mag1z(b:c));
    title('wheel 2 on, mag1z reading')

    %%% only wheel 3 on
    subplot(3,3,7)
    plot(curvefitx3,time(c:d),mag1x(c:d));
    title('wheel 3 on, mag1x reading')
    hold on
    subplot(3,3,8)
    plot(curvefity3,time(c:d),mag1y(c:d));
    title('wheel 3 on, mag1y reading')
    subplot(3,3,9)
    plot(curvefitz3,time(c:d),mag1z(c:d));
    title('wheel 3 on, mag1z reading')

%     %%%% driver noise data for 'testwheelnoise.csv' data 
%     figure(2)
%     [dcurvex,dgofx,doutputx] = fit(time(1:a),mag1x(1:a),'smoothingspline','normalize','on');
%     subplot(3,1,1)
%     plot(dcurvex,time(1:a),mag1x(1:a));
%     title('driver vibes x reading')
% 
%     [dcurvey,dgofy,doutputy] = fit(time(1:a),mag1y(1:a),'smoothingspline','normalize','on');
%     subplot(3,1,2)
%     plot(dcurvey,time(1:a),mag1y(1:a));
%     title('driver vibes y reading')
% 
%     [dcurvez,dgofz,doutputz] = fit(time(1:a),mag1z(1:a),'smoothingspline','normalize','on');
%     subplot(3,1,3)
%     plot(dcurvez,time(1:a),mag1z(1:a));
%     title('driver vibes z reading')


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
    % 
    % subplot(3,1,2)
    % plot(curvefity,time,mag1y);
    % hold on
    % plot(curvefitx,time,mag1y,'Residuals')
    % xlabel('time(s)'); ylabel('mag1y (T)')
    % %plot(curvefit,time,mag1y,'predfunc')
    % formula(curvefity)
    % py.mean = mean(mag1y); py.std = std(mag1y); py.var = var(mag1y);
    % py.max = max(mag1y);  py.min = min(mag1y);
    % disp(py)
    % 
    % subplot(3,1,3)
    % plot(curvefitz,time,mag1z);
    % hold on
    % plot(curvefitz,time,mag1z,'Residuals')
    % xlabel('time(s)'); ylabel('mag1z (T)')
    % %plot(curvefit,time,mag1x,'predfunc')
    % formula(curvefitz)
    % pz.mean = mean(mag1z); pz.std = std(mag1z); pz.var = var(mag1z);
    % pz.max = max(mag1z);  pz.min = min(mag1z);
    % disp(pz)
end