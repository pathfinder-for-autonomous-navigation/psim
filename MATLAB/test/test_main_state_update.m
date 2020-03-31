main_state = initialize_main_state(1,'default');
main_state = main_state_update(main_state);
assert(isstruct(main_state),'main_state is not a struct')

