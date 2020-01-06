addpath('test');
config()
runtests('test_initialize_main_state');
runtests('test_main_state_update');
runtests('test_dynamics_update');
runtests('test_sensors_update');
runtests('test_sensor_reading');
runtests('test_actuator_command');
runtests('test_initialize_computer_states');
runtests('test_quaternion_math');
runtests('test_get_truth');

runtests('test_env_sun_vector');
runtests('test_env_earth_attitude');
runtests('test_env_magnetic_field');
runtests('test_env_gravity');