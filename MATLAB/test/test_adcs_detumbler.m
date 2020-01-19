state = adcs_initialize_detumbler_state();
[state,magrod_moment_cmd] = adcs_detumbler(state,zeros(3,1));
assert(isstruct(state),'state is not a struct')
assert(all(size(magrod_moment_cmd) == [3,1]),'magrod_moment_cmd is not size [3,1]')
[state,magrod_moment_cmd] = adcs_detumbler(state,zeros(3,1));
assert(isstruct(state),'state is not a struct')
assert(all(size(magrod_moment_cmd) == [3,1]),'magrod_moment_cmd is not size [3,1]')

