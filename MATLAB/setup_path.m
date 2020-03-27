%Script to setup the matlab path.
[filepath, name, ext] = fileparts(mfilename("fullpath"));
addpath(genpath(filepath));

%this makes sure TrackingComponentLibrary path has its mex files
%alphabetically added to the top of the path
addpath(genpath(strcat(userpath+"/Add-Ons/Toolboxes/TrackerComponentLibrary")))
