%Script to compile everything, and get data files, only call this on fresh
%installs, or if you want to refresh everything.
setup_path();
%This doesn't work on linux for some reason.
% %compile TrackingComponentLibrary if needed.
% try 
%     TAI2UTC(2436934.5,1000);
% catch
%     CompileCLibraries;
% end
% %download latest earth orientation parameters
% getEOP(2444244.5,0,3,true);
%compile everything
generate_mex_code();