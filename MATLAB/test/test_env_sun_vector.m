%TEST_ENV_SUN_VECTOR Tests env_sun_vector.m
%   has an error if env_sun_vector is over 0.1 degrees off JPL model
global const

years_to_test= 5;
angle_error_margin= 0.1*pi/180;
norm_margin= 1E-6;
N= 20;
test_times= linspace(0.0,years_to_test*365*24*60*60,N);%test times, 20 years

for time = test_times
    
    earth2sun_truth = planetEphemeris(juliandate(utl_time2datetime(time,const.INITGPS_WN)),'Earth','Sun','421');
    earth2sun_truth = (earth2sun_truth/norm(earth2sun_truth))';
    earth2sun_test = env_sun_vector(time);
    
    assert(abs(norm(earth2sun_test)-1.0)<=norm_margin,"Sun vector model test failed not normalized");      
    cos_error= dot(earth2sun_test,earth2sun_truth)/norm(earth2sun_test)/norm(earth2sun_truth);
    assert(cos_error>=cos(angle_error_margin),"Sun vector model test failed");  
    
end

