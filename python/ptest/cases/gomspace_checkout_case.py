# Gomspace test case. Gets cycle count purely for diagnostic purposes
from .base import SingleSatOnlyCase


class GomspaceCheckoutCase(SingleSatOnlyCase):
    def setup_case_singlesat(self, simulation):
        simulation.flight_controller.write_state(
            "pan.state", 9)  # Manual state

    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state(
            "pan.cycle_no")


class GomspaceCheckoutSimCase(GomspaceCheckoutCase):
    @property
    def run_sim(self):
        return True
