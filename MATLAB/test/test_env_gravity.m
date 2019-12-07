clearvars;

global const

config();
rng(2,'threefry');
years_to_test= 5;
margin=1E-9; 
N= 20;
test_times= linspace(-years_to_test*365*24*60*60,0.0,N);%test times, 20 years


for time = test_times
    x=randn(3,1);
    x=x/norm(x)*(rand()*200E3+400E3+const.R_EARTH);
    g=zeros([3,1]);
    [g(1), g(2), g(3)] = gravitysphericalharmonic( x', 'EGM96', 30 );
    %now B is in NED coords, and must be transformed back to ECEF.
    g_test= env_gravity(time,x);
    %norm(g_test-g);
    assert(norm(g_test-g)<=margin,"gravity field out of margin");      
    
end

