%Script to setup the matlab path.
[filepath, name, ext] = fileparts(mfilename("fullpath"));
addpath(genpath(strcat(filepath, '/utl')));
addpath(genpath(strcat(filepath, '/environmental_models')));
addpath(genpath(strcat(filepath, '/plot')));
addpath(genpath(strcat(filepath, '/adcs')));
addpath(genpath(strcat(filepath, '/orbit_estimation')));
addpath(genpath(strcat(filepath, '/test')));
addpath(genpath(strcat(filepath, '/3rd_Party_Libraries')));

