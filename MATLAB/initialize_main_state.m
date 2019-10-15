function main_state = initialize_main_state(seed,condition)
%initialize_main_state Sample an initial main state given a seed and condition
%   To set the seed based on current time, use seed = 'shuffle'
global const

main_state=struct();
rng(seed,'threefry');
end

