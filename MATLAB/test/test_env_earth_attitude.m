%TEST_ENV_EARTH_ATTITUDE Tests env_earth_attitude.m
%   has an error if earth_attitude is over 0.1 degrees off matlab model
global const

years_to_test= 5;
angle_error_margin= 0.1*pi/180;%radians
rate_error_margin= 1E-16;%rad/s
norm_margin= 1E-6;
N= 100;
test_times= linspace(0.0,years_to_test*365*24*60*60,N);%test times

for time = test_times
    T=utl_time2datetime(time,const.INITGPS_WN);
    dcm=dcmeci2ecef('IAU-2000/2006',[year(T),month(T),day(T),hour(T),minute(T),second(T)]);
    q=dcm2quat(dcm);
    quat_ecef_eci(4)=q(1);
    quat_ecef_eci(1)=q(2);
    quat_ecef_eci(2)=q(3);
    quat_ecef_eci(3)=q(4);
    rate_ecef_truth=[0;0;7.2921158553E-5;];
    quat_ecef_eci_truth=quat_ecef_eci';
    [quat_ecef_eci_test,rate_ecef_test] = env_earth_attitude(time);
    
    assert(norm(rate_ecef_truth-rate_ecef_test)<=rate_error_margin,"rate error");  
    assert(norm(quat_ecef_eci_test)-1.0<=norm_margin,"quaternion not normalized");   
    cos_half_error= abs(dot(quat_ecef_eci_truth,quat_ecef_eci_test));
    
    assert(cos_half_error>=cos(angle_error_margin/2),"quaternions to far apart");  
    
end
