function plot_almost_conserved_values(main_state_trajectory)
%plot_almost_conserved_values creates plots of angular momentums in eci and total energies
% if these have a huge jump, there is a problem with physics.

plot_get_truth(main_state_trajectory,'total angular momentum follower (Nms) eci','f')
plot_get_truth(main_state_trajectory,'total angular momentum leader (Nms) eci','l')
plot_get_truth(main_state_trajectory,'orbital energy follower (J)','f')
plot_get_truth(main_state_trajectory,'orbital energy leader (J)','l')
plot_get_truth(main_state_trajectory,'rotational energy follower (J)','f')
plot_get_truth(main_state_trajectory,'rotational energy leader (J)','l')
plot_get_truth(main_state_trajectory,'orbital angular momentum follower (Nms) eci','f')
plot_get_truth(main_state_trajectory,'orbital angular momentum leader (Nms) eci','l')