%%% Body frame unit normals
%%% Leader F2, F3, F4, F5, F6
%%% Follower F2, F3, F4, F5, F6

% Need the following data sets:
% 'Face2+XFollower10011528raw_Data.csv'
% 'Face2+XFollower10012437raw_Data.csv'
% 'BFace2+XFollower10212928raw_Data.csv'
% 'Face3-YFollower09205506raw_Data.csv'
% 'Face3-YFollower09203820raw_Data.csv'
% 'BFace2+XFollower10212928raw_Data.csv'
% 'newFace4-XFollower11220604raw_Data.csv'
% 'newBFace4-XFollower11222209raw_Data.csv'
% 'Face3-YLeader16204111raw_Data.csv'
% 'BFace5+YFollower10220404raw_Data.csv'
% 'Face6+ZFollower11011312raw_Data.csv'
% 'BFace6+ZFollower11010110raw_Data.csv'
% 'Face2+XLeader13180055raw_Data.csv'
% 'BFace2+XLeader13180828raw_Data.csv'
% 'Face3-YLeader16204111raw_Data.csv'
% 'BFace3-YLeader16210220raw_Data.csv'
% 'Face4-XLeader16214521raw_Data.csv'
% 'BFace4-XLeader16215313raw_Data.csv'
% 'Face5+YLeader16213139raw_Data.csv'
% 'BFace5+YLeader16212326raw_Data.csv'
% 'Face6+ZLeader13212220raw_Data.csv'
% 'BFace6+ZLeader13212849raw_Data.csv'
    
clc
clear all
close all

normsL = {};
normsF = {};
global const
factors = 0.5; %0.1:0.01:1; %decided to keep 0.5 as the threshold
tallies = zeros(length(factors),1);
ers = zeros(length(factors),4);
count  = 0;

for i = factors
    const.factor = i;
    count = count + 1;
    
    % Follower Face 2 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    % 
    % 0, 1 use file 10011528
    % 2, 3 use file 10012437
    %
    %figure(1)
    y_data_1 = csvread('Face2+XFollower10011528raw_Data.csv');
    y_data_2 = csvread('Face2+XFollower10012437raw_Data.csv');
    x_data_1 = csvread('BFace2+XFollower10212928raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %%%sub%plot(4,1,1)
    %%plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %%sub%plot(4,1,2)
    %%plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                       [y_data_2(:, 2); x_data_1(:, 2)],...
                       [y_data_2(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %%plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                       [y_data_2(:, 2); x_data_1(:, 2)],...
                       [y_data_2(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %%sub%plot(4,1,4)
    %%plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsF{2,1} = n0;
    normsF{2,2} = n1;
    normsF{2,3} = n2;
    normsF{2,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1;

    % Follower Face 3 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    % 
    % 0, 1 use both
    % 3 use 09205506
    % 2 use 09203820
    %
    y_data_1 = csvread('Face3-YFollower09205506raw_Data.csv');
    y_data_2 = csvread('Face3-YFollower09203820raw_Data.csv');
    x_data_1 = csvread('BFace2+XFollower10212928raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); y_data_2(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); y_data_2(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); y_data_2(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %%sub%plot(4,1,1)
    %%plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); y_data_2(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); y_data_2(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); y_data_2(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %%sub%plot(4,1,2)
    %%plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                       [y_data_2(:, 2); x_data_1(:, 2)],...
                       [y_data_2(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %%plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %%sub%plot(4,1,4)
    %%plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsF{3,1} = n0;
    normsF{3,2} = n1;
    normsF{3,3} = n2;
    normsF{3,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 

    % Follower Face 4 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    % It was first thought that the board was mounted improperly. Upon further
    % inspection and a second test setup, however, the test results were
    % confirmed to be correct.
    %
    y_data_1 = csvread('newFace4-XFollower11220604raw_Data.csv');
    x_data_1 = csvread('newBFace4-XFollower11222209raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %%sub%plot(4,1,1)
    %%plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %%sub%plot(4,1,2)
    %%plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %%plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %%sub%plot(4,1,4)
    %%plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsF{4,1} = n0;
    normsF{4,2} = n1;
    normsF{4,3} = n2;
    normsF{4,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 
    
    % Follower Face 5 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    % Much smaller response on one of the vectors
    % The entire sun sensor was rotated about the normal vector of the board
    %
    y_data_1 = csvread('Face3-YLeader16204111raw_Data.csv');
    x_data_1 = csvread('BFace5+YFollower10220404raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %%sub%plot(4,1,1)
    %%plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %%sub%plot(4,1,2)
    %%plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %%plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %%sub%plot(4,1,4)
    %%plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsF{5,1} = n0;
    normsF{5,2} = n1;
    normsF{5,3} = n2;
    normsF{5,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 

    % Follower Face 6 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face6+ZFollower11011312raw_Data.csv');
    x_data_1 = csvread('BFace6+ZFollower11010110raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %%subplot(4,1,1)
    %%plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %subplot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsF{6,1} = n0;
    normsF{6,2} = n1;
    normsF{6,3} = n2;
    normsF{6,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 

    % Leader Face 2 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face2+XLeader13180055raw_Data.csv');
    x_data_1 = csvread('BFace2+XLeader13180828raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %subplot(4,1,1)
    %plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %subplot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsL{2,1} = n0;
    normsL{2,2} = n1;
    normsL{2,3} = n2;
    normsL{2,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 
    % Leader Face 3 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face3-YLeader16204111raw_Data.csv');
    x_data_1 = csvread('BFace3-YLeader16210220raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %subplot(4,1,1)
    %plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsL{3,1} = n0;
    normsL{3,2} = n1;
    normsL{3,3} = n2;
    normsL{3,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 
    % Leader Face 4 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face4-XLeader16214521raw_Data.csv');
    x_data_1 = csvread('BFace4-XLeader16215313raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %subplot(4,1,1)
    %plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsL{4,1} = n0;
    normsL{4,2} = n1;
    normsL{4,3} = n2;
    normsL{4,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 
    
    % Leader Face 5 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face5+YLeader16213139raw_Data.csv');
    x_data_1 = csvread('BFace5+YLeader16212326raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %subplot(4,1,1)
    %plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %%sub%plot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsL{5,1} = n0;
    normsL{5,2} = n1;
    normsL{5,3} = n2;
    normsL{5,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;

count  = 0

for i = factors
    const.factor = i;
    count = count + 1; 
    
    % Leader Face 6 Normal Calculations
    %
    % x degress, y degress, v0, v1, v2, v3
    %
    y_data_1 = csvread('Face6+ZLeader13212220raw_Data.csv');
    x_data_1 = csvread('BFace6+ZLeader13212849raw_Data.csv');
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 3); x_data_1(:, 3)]);
    [n0,R] = least_squares(S, v);
    %subplot(4,1,1)
    %plot(R'*v,n0)
    [fit1,gof,fitinfo] = fit(R'*v,n0,'poly1');
    ers(count,1) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 4); x_data_1(:, 4)]);
    [n1,R] = least_squares(S, v);
    %subplot(4,1,2)
    %plot(R'*v,n1)
    [fit1,gof,fitinfo] = fit(R'*v,n1,'poly1');
    ers(count,2) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 5); x_data_1(:, 5)]);
    [n2,R] = least_squares(S, v);
    %subplot(4,1,3)
    %plot(R'*v,n2)
    [fit1,gof,fitinfo] = fit(R'*v,n2,'poly1');
    ers(count,3) = gof.rmse;
    
    [S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                       [y_data_1(:, 2); x_data_1(:, 2)],...
                       [y_data_1(:, 6); x_data_1(:, 6)]);
    [n3,R] = least_squares(S, v);
    %subplot(4,1,4)
    %plot(R'*v,n3)
    [fit1,gof,fitinfo] = fit(R'*v,n3,'poly1');
    ers(count,4) = gof.rmse;
    
    normsL{6,1} = n0;
    normsL{6,2} = n1;
    normsL{6,3} = n2;
    normsL{6,4} = n3;
end

[M I] = min(ers(:,1));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,2));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,3));
tallies(I) = tallies(I) + 1;
[M I] = min(ers(:,4));
tallies(I) = tallies(I) + 1;


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
    global const
    thresh = max(voltages)*const.factor;
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