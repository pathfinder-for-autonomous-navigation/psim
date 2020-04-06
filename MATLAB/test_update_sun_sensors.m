config()
main_state = initialize_main_state(1,'default');
sensor_readings_follower = sensor_reading(main_state.follower,main_state.leader);
sensor_readings_leader = sensor_reading(main_state.leader,main_state.follower);

mus = zeros(100,1); %stores means for the random test cases
sigs = zeros(100,1); %stores the standard deviations for the random test cases

%%%test with random sat2sun_body vectors
for k = 1:100
    s = randn(3,1);
    v = s/norm(s);
    [sun_vec, success] = update_sun_sensors(main_state.follower.sensors, v, 1);
    %mus(k) = mu; sigs(k) = sig;
end
