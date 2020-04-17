%config()
main_state = initialize_main_state(1,'default');
sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);
assert(isstruct(sensor_readings_follower),'sensor_readings_follower is not a struct')
assert(isstruct(sensor_readings_leader),'sensor_readings_leader is not a struct')
assert(sensor_readings_follower.sun_sensor_true == 1, 'sun_vec not generated for follower')
assert(sensor_readings_leader.sun_sensor_true == 1, 'sun_vec not generated for leader')

n = 50; %number of tests
mus = zeros(n,1); %stores means of voltages for the random test cases
sigs = zeros(n,1); %stores the standard deviations (noise) of voltages for the random test cases
offset_angles = zeros(n,1); %offset angle between input and output sun_vec

%%%test with random sat2sun_body vectors
for k = 1:n
    s = randn(3,1);
    v = s/norm(s);
    [sun_vec, success,mu,sig] = update_sun_sensors(main_state.follower.sensors, v, 0);
    mus(k) = mu; sigs(k) = sig; %to get sun sensor noise characteristics
    offset_angles(k) = acosd(dot(v,sun_vec)/(norm(v)*norm(sun_vec)));
end

%%get stats for offset angles
% s = rmmissing(offset_angles);
% std(s); mean(s); min(s); max(s);

% figure(5)
% histogram(offset_angles,20)
% xlabel('theta (degrees)'); ylabel('count')
% title('offset angles counter');
% saveas(gcf,'offset angles counter.png')