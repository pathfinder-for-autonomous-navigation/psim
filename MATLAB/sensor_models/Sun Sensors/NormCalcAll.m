%%% Body frame unit normals
%%% Leader F2, F3, F4, F5, F6
%%% Follower F2, F3, F4, F5, F6
clc
clear all
close all

%%%Follower Files

y_datas = {'newFace4-XFollower11220604raw_Data.csv', 'Face3-YLeader16204111raw_Data.csv',...
    'Face6+ZFollower11011312raw_Data.csv'};
x_datas = {'newBFace4-XFollower11222209raw_Data.csv','BFace5+YFollower10220404raw_Data.csv',...
    'BFace6+ZFollower11010110raw_Data.csv'};
Cs_F = {};
count  = 0;

for f = 2:6 %loops through Faces
    
    if f == 2
        % Follower Face 2 Normal Calculations
        % x degress, y degress, v0, v1, v2, v3
        % 0, 1 use file 10011528
        % 2, 3 use file 10012437
        
        y_data_1 = csvread('Face2+XFollower10011528raw_Data.csv');
        y_data_2 = csvread('Face2+XFollower10012437raw_Data.csv');
        x_data_1 = csvread('BFace2+XFollower10212928raw_Data.csv');
        
        %%%%%%% photodiode 1
        [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 3); x_data_1(:, 3)]);
        [n0,R] = least_squares(S, v);
        thetas_measured = acosd(S*n0);
        
        n0 = n0/norm(n0);
        thetas_measured = acosd(S*n0);
        
        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{2,1} = Cs;
        saveas(gcf,'fitted_voltagesF2_P1.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%%%% photodiode 2
        [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 4); x_data_1(:, 4)]);
        [n1,R] = least_squares(S, v);

        n1 = n1/norm(n1);
        thetas_measured = acosd(S*n1);
        
        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{2,2} = Cs;
        saveas(gcf,'fitted_voltagesF2_P2.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%%%% photodiode 3
        [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                           [y_data_2(:, 2); x_data_1(:, 2)],...
                           [y_data_2(:, 5); x_data_1(:, 5)]);
        [n2,R] = least_squares(S, v);

        n2 = n2/norm(n2);
        thetas_measured = acosd(S*n2);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{2,3} = Cs;
        saveas(gcf,'fitted_voltagesF2_P3.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%%%% photodiode 4
        [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                           [y_data_2(:, 2); x_data_1(:, 2)],...
                           [y_data_2(:, 6); x_data_1(:, 6)]);
        [n3,R] = least_squares(S, v);
        
        n3 = n3/norm(n3);
        thetas_measured = acosd(S*n3);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{2,4} = Cs;
        saveas(gcf,'fitted_voltagesF2_P4.png')
        close all
        count = count + 1;
        disp(count)
    
    elseif f == 3
        % Follower Face 3 Normal Calculations
        % x degress, y degress, v0, v1, v2, v3
        % 0, 1 use both
        % 3 use 09205506
        % 2 use 09203820
        y_data_1 = csvread('Face3-YFollower09205506raw_Data.csv');
        y_data_2 = csvread('Face3-YFollower09203820raw_Data.csv');
        x_data_1 = csvread('BFace2+XFollower10212928raw_Data.csv');
        
        %%%%%%% photodiode 1
        [S, v] = trim_data([y_data_1(:, 1); y_data_2(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); y_data_2(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 3); y_data_2(:, 3); x_data_1(:, 3)]);
        [n0,R] = least_squares(S, v);
        
        n0 = n0/norm(n0);
        thetas_measured = acosd(S*n0);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{3,1} = Cs;
        saveas(gcf,'fitted_voltagesF3_P1.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%% photodiode 2
        
        [S, v] = trim_data([y_data_1(:, 1); y_data_2(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); y_data_2(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 4); y_data_2(:, 4); x_data_1(:, 4)]);
        [n1,R] = least_squares(S, v);
        
        n1 = n1/norm(n1);
        thetas_measured = acosd(S*n1);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{3,2} = Cs;
        saveas(gcf,'fitted_voltagesF3_P2.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%%photodiode 3
        
        [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                           [y_data_2(:, 2); x_data_1(:, 2)],...
                           [y_data_2(:, 5); x_data_1(:, 5)]);
        [n2,R] = least_squares(S, v);
        
        n2 = n2/norm(n2);
        thetas_measured = acosd(S*n2);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{3,3} = Cs;
        saveas(gcf,'fitted_voltagesF3_P3.png')
        close all
        count = count + 1;
        disp(count)
        
        %%%%%photodiode 4
        [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 6); x_data_1(:, 6)]);
        [n3,R] = least_squares(S, v);
        
        n3 = n3/norm(n3);
        thetas_measured = acosd(S*n3);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_F{3,4} = Cs;
        saveas(gcf,'fitted_voltagesF3_P4.png') 
        close all
        count = count + 1;
        disp(count)
    
    else
        y_data_1 = csvread(y_datas{f-3});
        x_data_1 = csvread(x_datas{f-3});
        
        for p = 1:4 %loops through the photodiodes
            
            [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                               [y_data_1(:, 2); x_data_1(:, 2)],...
                               [y_data_1(:, 2+p); x_data_1(:, 2+p)]);
            [n,R] = least_squares(S, v);

            n = n/norm(n);
            thetas_measured = acosd(S*n);

            %using the measured voltages, do a nonlinear fitting
            voltages = v;
            fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                                Cs(3)*thetas_measured.^2 +...
                                Cs(4)*thetas_measured.^4);

            opts = optimset('Display','off');
            Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                          0.01,0.1],thetas_measured,voltages,[],[],opts);

            %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
            scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                                Cs(3)*thetas_measured.^2 +...
                                (Cs(4)*thetas_measured.^4));

            fitted_voltages = scaled_fun(Cs,thetas_measured);
            plot(thetas_measured, fitted_voltages)
            hold on
            scatter(thetas_measured, voltages)
            xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
            Cs_F{f,p} = Cs;
            saveas(gcf,['fitted_voltagesF' num2str(f) '_P'...
                num2str(p) '.png'])
            close all
            count = count + 1;
            disp(count)
        end
    end
end

%%%Leader Files
y_datas = {'Face2+XLeader13180055raw_Data.csv','Face3-YLeader16204111raw_Data.csv'...
    'Face4-XLeader16214521raw_Data.csv','Face5+YLeader16213139raw_Data.csv',...
    'Face6+ZLeader13212220raw_Data.csv'};
x_datas = {'BFace2+XLeader13180828raw_Data.csv','BFace3-YLeader16210220raw_Data.csv'...
    'BFace4-XLeader16215313raw_Data.csv','BFace5+YLeader16212326raw_Data.csv',...
    'BFace6+ZLeader13212849raw_Data.csv'};
Cs_L = {};

for f = 2:6 %loops through Faces

    y_data_1 = csvread(y_datas{f-1});
    x_data_1 = csvread(x_datas{f-1});

    for p = 1:4 %loops through the photodiodes

        [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                           [y_data_1(:, 2); x_data_1(:, 2)],...
                           [y_data_1(:, 2+p); x_data_1(:, 2+p)]);
        [n,R] = least_squares(S, v);

        n = n/norm(n);
        thetas_measured = acosd(S*n);

        %using the measured voltages, do a nonlinear fitting
        voltages = v;
        fun = @(Cs,thetas_measured)(Cs(1) + Cs(2)*cos(thetas_measured) + ...
                            Cs(3)*thetas_measured.^2 +...
                            Cs(4)*thetas_measured.^4);

        opts = optimset('Display','off');
        Cs = lsqcurvefit(fun,[0.4,max(voltages),...
                      0.01,0.1],thetas_measured,voltages,[],[],opts);

        %voltages scaled: Vmax_data = 3.3, ~0.3 for under the lamp
        scaled_fun = @(Cs,thetas_measured)(Cs(1) + (Cs(2)*cos(thetas_measured)) + ...
                            Cs(3)*thetas_measured.^2 +...
                            (Cs(4)*thetas_measured.^4));

        fitted_voltages = scaled_fun(Cs,thetas_measured);
        plot(thetas_measured, fitted_voltages)
        hold on
        scatter(thetas_measured, voltages)
        xlabel('thetas_measured (deg)'); ylabel('fitted_voltages (V)');
        Cs_L{f,p} = Cs;
        saveas(gcf,['fitted_voltagesL' num2str(f) '_P'...
                num2str(p) '.png'])
        close all
        count = count + 1;
        disp(count)
    end
end

function plot_normals(n0, n1, n2, n3)
    figure
    hold on
    plot3([0 n0(1)], [0 n0(2)], [0 n0(3)], '-b')
    plot3([0 n1(1)], [0 n1(2)], [0 n1(3)], '-g')
    plot3([0 n2(1)], [0 n2(2)], [0 n2(3)], '-k')
    plot3([0 n3(1)], [0 n3(2)], [0 n3(3)], '-r')
    n = [0.0; 0.0; -1.0];
    nn = x_rot(-20) * n;
    plot3([0 nn(1)], [0 nn(2)], [0 nn(3)], '-c')
    nn = x_rot(20) * n;
    plot3([0 nn(1)], [0 nn(2)], [0 nn(3)], '-c')
    nn = y_rot(-20) * n;
    plot3([0 nn(1)], [0 nn(2)], [0 nn(3)], '-c')
    nn = y_rot(20) * n;
    plot3([0 nn(1)], [0 nn(2)], [0 nn(3)], '-c')
    hold off
end

function [n,R] = least_squares(S, v)
    [Q, R] = qr(S);
    n = R \ (transpose(Q) * v);
end
function [S, v] = trim_data(x_angle, y_angle, voltages)
    thresh = max(voltages)*0.5;
    [r, ~] = size(x_angle);
    S = zeros(r, 3);
    v = zeros(r, 1);
    i = 0;
    for j = 1:r
        if voltages(j) > thresh
            i = i + 1;
            if abs(x_angle(j) > y_angle(j))
                S(i, :) = (x_rot(-x_angle(j)) * (y_rot(-y_angle(j)) * [0.0; 0.0; -1.0]))';
            else
                S(i, :) = ((y_rot(-y_angle(j)) * x_rot(-x_angle(j)) * [0.0; 0.0; -1.0]))';
            end
            v(i) = voltages(j);
        end
    end
    S = S(1:i, :);
    v = v(1:i);
end
function M = x_rot(angle)
    M = [1 0 0
         0 cosd(angle) -sind(angle)
         0 sind(angle) cosd(angle)];
end
function M = y_rot(angle)
    M = [cosd(angle) 0 sind(angle)
        0 1 0
        -sind(angle) 0 cosd(angle)];
end