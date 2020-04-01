%% Follower Face 2 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
% 
% 0, 1 use file 10011528
% 2, 3 use file 10012437
%
y_data_1 = csvread('Face2+XFollower10011528raw_Data.csv');
y_data_2 = csvread('Face2+XFollower10012437raw_Data.csv');
x_data_1 = csvread('BFace2+XFollower10212928raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                   [y_data_2(:, 2); x_data_1(:, 2)],...
                   [y_data_2(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                   [y_data_2(:, 2); x_data_1(:, 2)],...
                   [y_data_2(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Follower Face 3 Normal Calculations
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
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); y_data_2(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); y_data_2(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); y_data_2(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_2(:, 1); x_data_1(:, 1)],...
                   [y_data_2(:, 2); x_data_1(:, 2)],...
                   [y_data_2(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Follower Face 4 Normal Calculations
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
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Follower Face 5 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
%
% Much smaller response on one of the vectors
% The entire sun sensor was rotated about the normal vector of the board
%
y_data_1 = csvread('Face5+YFollower09232347raw_Data.csv');
x_data_1 = csvread('BFace5+YFollower10220404raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Follower Face 6 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
%
y_data_1 = csvread('Face6+ZFollower11011312raw_Data.csv');
x_data_1 = csvread('BFace6+ZFollower11010110raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Spare A Face 2 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
%
y_data_1 = csvread('Face2+XSA13171714raw_Data.csv');
x_data_1 = csvread('BFace2+XSA13170315raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Spare B Face 2 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
%
y_data_1 = csvread('Face2+XSB13183340raw_Data.csv');
x_data_1 = csvread('BFace2+XSB13182446raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
%% Leader Face 2 Normal Calculations
%
% x degress, y degress, v0, v1, v2, v3
%
y_data_1 = csvread('Face2+XLeader13180055raw_Data.csv');
x_data_1 = csvread('BFace2+XLeader13180828raw_Data.csv');
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 3); x_data_1(:, 3)]);
n0 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 4); x_data_1(:, 4)]);
n1 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 5); x_data_1(:, 5)]);
n2 = least_squares(S, v);
[S, v] = trim_data([y_data_1(:, 1); x_data_1(:, 1)],...
                   [y_data_1(:, 2); x_data_1(:, 2)],...
                   [y_data_1(:, 6); x_data_1(:, 6)]);
n3 = least_squares(S, v);
plot_normals(n0, n1, n2, n3)
grid on
norm(n0)
norm(n1)
norm(n2)
norm(n3)
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
function n = least_squares(S, v)
    [Q, R] = qr(S);
    n = R \ (transpose(Q) * v);
end
function [S, v] = trim_data(x_angle, y_angle, voltages)
    thresh = max(voltages) / 2.0;
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