[filepath,~,~] = fileparts(mfilename('fullpath'));
geomag_incl = strcat('-I', fullfile(filepath, "../src/inl"));
geograv_incl = strcat('-I', fullfile(filepath, "../geograv/include"));

lin_incl = strcat('-I', fullfile(filepath, "../lib/lin/include"));
gnc_incl = strcat('-I', fullfile(filepath, "../include"));
conversion_incl = strcat('-I', fullfile(filepath, "attitude_filter"));

geomag_file = fullfile(filepath, "environmental_models/helper_functions/geomag_wrapper.cpp");
geograv_file = fullfile(filepath, "environmental_models/helper_functions/geograv_wrapper.cpp");

% for attitude estimator
gnc_env_file = fullfile(filepath, "../src/gnc_environment.cpp");
gnc_attitude_estimator_file = fullfile(filepath, "../src/gnc_attitude_estimator.cpp");
gnc_constants_file = fullfile(filepath, "../src/gnc_constants.cpp");
% conversions_file = fullfile(filepath, "attitude_filter/helper_functions/conversion.h");
estimator_reset_file = fullfile(filepath, "attitude_filter/helper_functions/estimator_reset.cpp");
estimator_update_file = fullfile(filepath, "attitude_filter/helper_functions/estimator_update.cpp");

try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geomag_file, geomag_incl); catch; end
try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geograv_file, geograv_incl); catch; end

try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14 -D GNC_NO_CASSERT", estimator_reset_file, gnc_attitude_estimator_file, gnc_env_file, gnc_constants_file, conversion_incl, gnc_incl, lin_incl); catch; end
try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14 -D GNC_NO_CASSERT", estimator_update_file, gnc_attitude_estimator_file, gnc_env_file, gnc_constants_file, conversion_incl, gnc_incl, lin_incl); catch; end

% try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geograv_file, geograv_incl); catch; end
