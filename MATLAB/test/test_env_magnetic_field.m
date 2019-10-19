clearvars;

global const

config();
rng(2,'threefry');
years_to_test= 2;
margin=10E-9; %10 nT error
N= 20;
test_times= linspace(-years_to_test*365*24*60*60,0.0,N);%test times, 20 years

for time = test_times
    x=randn(3,1);
    x=x/norm(x)*(rand()*500E3+const.R_EARTH);
    B=zeros([3,1]);
    x_lla= ecef2lla(x')';
    [xyz,~,~,~,~]= wrldmagm(x_lla(3), x_lla(1), x_lla(2), decyear(utl_time2datetime(time,const.INITGPS_WN)), '2015v2');
    %now B is in NED coords, and must be transformed back to ECEF.
    [B(1),B(2),B(3)]= ned2ecefv(xyz(1),xyz(2),xyz(3),x_lla(1),x_lla(2));
    B= 1E-9*B;
    B_test= env_magnetic_field(time,x);
    assert(norm(B_test-B)<=margin,"Magnetic field out of margin");      
    
end

