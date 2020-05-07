[filepath,~,~] = fileparts(mfilename('fullpath'));
geomag_incl = strcat('-I', fullfile(filepath, "../src/inl"));
geograv_incl = strcat('-I', fullfile(filepath, "../geograv/include"));

lin_incl = strcat('-I', fullfile(filepath, "../lib/lin/include"));
gnc_incl = strcat('-I', fullfile(filepath, "../include"));

geomag_file = fullfile(filepath, "environmental_models/helper_functions/geomag_wrapper.cpp");
geograv_file = fullfile(filepath, "environmental_models/helper_functions/geograv_wrapper.cpp");

% for attitude estimator
gnc_env_file = fullfile(filepath, "../src/gnc_environment.cpp");
gnc_attitude_estimator_file = fullfile(filepath, "../src/gnc_attitude_estimator.cpp");
gnc_constants_file = fullfile(filepath, "../src/gnc_constants.cpp");
estimator_reset_file = fullfile(filepath, "attitude_filter/estimator_reset.cpp");
% geograv_file = fullfile(filepath, "environmental_models/helper_functions/geograv_wrapper.cpp");


try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geomag_file, geomag_incl); catch; end
try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geograv_file, geograv_incl); catch; end

mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", estimator_reset_file, gnc_attitude_estimator_file, gnc_env_file, gnc_constants_file, gnc_incl, lin_incl);

% try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geograv_file, geograv_incl); catch; end
