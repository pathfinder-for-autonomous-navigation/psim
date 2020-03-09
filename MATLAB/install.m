%Script to compile everything, and get data files, only call this on fresh
%installs, or if you want to refresh everything.
setup_path();
%compile everything
generate_mex_code();
%load in EOP
getEOP(2444244.5,0,3,true);