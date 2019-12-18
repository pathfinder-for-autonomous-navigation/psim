# Empty test case

class EmptyCase(object):
    single_sat_sim_compatible = True

    @staticmethod
    def case_runner(simulation):
        # Get cycle count purely for diagnostic purposes

        if simulation.is_single_sat_sim:
            simulation.cycle_no = simulation.flight_controller.read_state("pan.cycle_no")
        else:
            simulation.cycle_no_follower = simulation.flight_controller_follower.read_state("pan.cycle_no")
            simulation.cycle_no_leader = simulation.flight_controller_leader.read_state("pan.cycle_no")
