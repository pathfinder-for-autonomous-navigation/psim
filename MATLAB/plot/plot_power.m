function plot_power(main_state_trajectory)
%plot_power plots power stuff.

plot_get_truth(main_state_trajectory,'solar panel area in sun follower (m^2)','f')
plot_get_truth(main_state_trajectory,'solar panel area in sun leader (m^2)','l')