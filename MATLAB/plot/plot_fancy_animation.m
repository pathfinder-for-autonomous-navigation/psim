function plot_fancy_animation(main_state_trajectory)
%PLOT_FANCY_ANIMATION creates a fancy animation from the simulation data.
%uses the cesium viewer saves "psim.html" file.
vector_names=["sat2sun","antenna","docking face","magnetic field","position"];
color_names=["r","b","g","y","k"];
czml= main_state_trajectory2czml_struct(main_state_trajectory,vector_names,color_names);
czml2html(czml,"psim.html",true);
end

