state = adcs_initialize_mag_bias_est();
state = adcs_mag_bias_est(state,0,0,[1;0;0]);
assert(isstruct(state),'state is not a struct')
assert(all(size(state.b_bias_est) == [3,1]),'state.b_bias_est is not size [3,1]')

state = adcs_mag_bias_est(state,0,0,[1;0;0]);
assert(isstruct(state),'state is not a struct')
assert(all(size(state.b_bias_est) == [3,1]),'state.b_bias_est is not size [3,1]')
