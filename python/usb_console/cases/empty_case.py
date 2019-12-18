# Empty test case. Gets cycle count purely for diagnostic purposes
from .base import FlexibleSimCase

class EmptyCase(FlexibleSimCase):
    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state("pan.cycle_no")

    def run_case_fullmission(self, simulation):
        simulation.cycle_no_follower = simulation.flight_controller_follower.read_state("pan.cycle_no")
        simulation.cycle_no_leader = simulation.flight_controller_leader.read_state("pan.cycle_no")
