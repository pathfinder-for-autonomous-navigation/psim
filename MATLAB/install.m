%Script to compile everything, and get data files, only call this on fresh
%installs, or if you want to refresh everything.
setup_path();
%compile TrackingComponentLibrary if needed.
try 
    TAI2UTC(2436934.5,1000);
catch
    CompileCLibraries;
end
%compile everything
generate_mex_code();