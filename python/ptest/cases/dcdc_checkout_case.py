from .base import SingleSatOnlyCase

class DCDCCheckcoutCase(SingleSatOnlyCase):

    def setup_case_singlesat(self, simulation):
        simulation.flight_controller.write_state(
            "pan.state", 9)  # Manual state
        self.run_case_singlesat(simulation)
        print("DCDC cases finished.")

    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state("pan.cycle_no")

        # writable command fields

        simulation.flight_controller.read_state("dcdc.ADCSMotor")
        simulation.flight_controller.read_state("dcdc.SpikeDock")
        simulation.flight_controller.read_state("dcdc.disable_cmd")
        simulation.flight_controller.read_state("dcdc.reset_cmd")