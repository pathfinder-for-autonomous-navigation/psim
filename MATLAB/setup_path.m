%Script to setup the matlab path.
[filepath, name, ext] = fileparts(mfilename("fullpath"));
addpath(genpath(filepath));
