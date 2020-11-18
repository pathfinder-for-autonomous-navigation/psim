clc; close all; clear all;

% From empirical test: ThrustVectorTest_2018_Nov_11_15h37m36s_300Firings_293K_refVac
fig = openfig('Impulse_vs_OnTime.fig');
axObjs = fig.Children;
dataObjs = axObjs.Children;

% Do a least squares linear regression on the dataset
time = dataObjs(1).XData; %on time; s
impulse = dataObjs(1).YData; %N-s; total impulse; aka change in momentum
c = polyfit(time,impulse,1);

disp(['Equation is y = ' num2str(c(1)) '*x + ' num2str(c(2))]);