function  test_env_sun_vector()
%TEST_ENV_SUN_VECTOR Tests env_sun_vector.m
%   has an error if env_sun_vector is over 0.1 degrees off JPL model

clearvars; clc;

global const

addpath('utl');
addpath('environmental_models');
addpath('environmental_models/helper_functions');

config();

years_to_test= 10;
angle_error_margin= 0.1*pi/180;
N= 100;
test_times= linspace(0.0,years_to_test*365*24*60*60,N);%test times, 20 years
error_vec = zeros(length(test_times),1);

for i = 1:length(test_times)
    
    time = test_times(i);
    
    fprintf('%.3f / %.3f \n',time,test_times(end))
    
    earth2sun_truth = planetEphemeris(juliandate(utl_time2datetime(time,const.INITGPS_WN)),'Earth','Sun');
    earth2sun_truth = (earth2sun_truth/norm(earth2sun_truth))';
    earth2sun_test = env_sun_vector(time);
    
    assert(norm(earth2sun_test)>=0.999,"Sun vector model test failed not normalized");  
    assert(norm(earth2sun_test)<=1.001,"Sun vector model test failed not normalized");   
    
    cos_error= dot(earth2sun_test,earth2sun_truth)/norm(earth2sun_test)/norm(earth2sun_truth);
    
    assert(cos_error>=cos(angle_error_margin),"Sun vector model test failed");  
    
    error_vec(i) = cos_error;
    
end

plot(test_times,error_vec); hold on
plot(test_times,repmat(cos(angle_error_margin),length(test_times)))
xlabel('time')
ylabel('cos(angle)')
legend('cos(error)')
legend('cos(angle_error_margin)')

