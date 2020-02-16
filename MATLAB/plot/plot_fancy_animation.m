function plot_fancy_animation(main_state_trajectory)
%PLOT_FANCY_ANIMATION creates a fancy animation from the simulation data.
%uses the cesium viewer
czml2html(main_state_trajectory2czml_struct(main_state_trajectory),"psim.html")
end

