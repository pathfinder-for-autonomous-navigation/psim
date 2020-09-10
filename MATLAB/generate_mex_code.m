[filepath,~,~] = fileparts(mfilename('fullpath'));
geomag_incl = strcat('-I', fullfile(filepath, "../src/inl"));
geograv_incl = strcat('-I', fullfile(filepath, "../geograv/include"));

geomag_file = fullfile(filepath, "environmental_models/helper_functions/geomag_wrapper.cpp");
geograv_file = fullfile(filepath, "environmental_models/helper_functions/geograv_wrapper.cpp");

try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geomag_file, geomag_incl); catch; end
try mex("-R2018a", "CXXFLAGS=$CXXFLAGS -std=c++14", geograv_file, geograv_incl); catch; end
