function plot_fancy_animation(main_state_trajectory)
%PLOT_FANCY_ANIMATION creates a fancy animation from the simulation data.
%uses the cesium viewer and saves a "psim.html" file.
% The pan model is from Microsoft PowerPoint.
% The template.html file is a combination of a lot of examples from cesium.
%       see https://sandcastle.cesium.com/
%           and https://cesium.com/docs/tutorials/cesium-workshop/
% CZML is a json file format described: 
%   https://github.com/AnalyticalGraphicsInc/czml-writer/wiki/CZML-Structure
%

% Started by Nathan Zimmerberg on Feb 16, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Feb 16, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University

%these are vector names from the `get_truth` function to animate at each sat
vector_names=["sat2sun","antenna","docking face","magnetic field","position"];
%these are color names for each vector, they are like the matlab plot colors codes
color_names=["r","b","g","y","k"];
%get the czml struct cell array, basically a json see 
%https://github.com/AnalyticalGraphicsInc/czml-writer/wiki/CZML-Structure
czml= main_state_trajectory2czml_struct(main_state_trajectory,vector_names,color_names);
czml2html(czml,"psim.html",true);
end

