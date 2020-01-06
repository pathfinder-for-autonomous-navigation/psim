global const
rng(2,'threefry');
years_to_test= 2;
margin=5E-9; %10 nT error
N= 200;
start_time= (2020.1-const.INIT_DYEAR)*365*24*60*60;
test_times= linspace(start_time,years_to_test*365*24*60*60,N);%test times, 20 years

for time = test_times
    x=randn(3,1);
    x=x/norm(x)*(rand()*500E3+const.R_EARTH);
    B=zeros([3,1]);
    x_lla= ecef2lla(x')';
    [xyz,~,~,~,~]= wrldmagm(x_lla(3), x_lla(1), x_lla(2), decyear(utl_time2datetime(time,const.INITGPS_WN)),'Custom', 'WMM2020.COF');
    %now B is in NED coords, and must be transformed back to ECEF.
    [B(1),B(2),B(3)]= ned2ecefv(xyz(1),xyz(2),xyz(3),x_lla(1),x_lla(2));
    B= 1E-9*B;
    B_test= env_magnetic_field(time,x);
    assert(norm(B_test-B)<=margin,"Magnetic field out of margin");      
    
end

