# Gomspace test case. Gets cycle count purely for diagnostic purposes and logs
# any other Gomspace state fields.
from .base import SingleSatOnlyCase


class GomspaceCheckoutCase(SingleSatOnlyCase):

    def setup_case_singlesat(self, simulation):
        simulation.flight_controller.write_state(
            "pan.state", 9)  # Manual state
        self.run_case_singlesat(simulation)
        print("Gomspace cases finished.")

    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state(
            "pan.cycle_no")

        # readable fields

        simulation.flight_controller.read_state("gomspace.vboost.output1")
        simulation.flight_controller.read_state("gomspace.vboost.output2")
        simulation.flight_controller.read_state("gomspace.vboost.output3")

        simulation.flight_controller.read_state("gomspace.vbatt")

        simulation.flight_controller.read_state("gomspace.curin.output1")
        simulation.flight_controller.read_state("gomspace.curin.output2")
        simulation.flight_controller.read_state("gomspace.curin.output3")

        simulation.flight_controller.read_state("gomspace.cursun")
        simulation.flight_controller.read_state("gomspace.cursys")

        simulation.flight_controller.read_state("gomspace.curout.output1")
        simulation.flight_controller.read_state("gomspace.curout.output2")
        simulation.flight_controller.read_state("gomspace.curout.output3")
        simulation.flight_controller.read_state("gomspace.curout.output4")
        simulation.flight_controller.read_state("gomspace.curout.output5")
        simulation.flight_controller.read_state("gomspace.curout.output6")

        simulation.flight_controller.read_state("gomspace.output.output1")
        simulation.flight_controller.read_state("gomspace.output.output2")
        simulation.flight_controller.read_state("gomspace.output.output3")
        simulation.flight_controller.read_state("gomspace.output.output4")
        simulation.flight_controller.read_state("gomspace.output.output5")
        simulation.flight_controller.read_state("gomspace.output.output6")

        simulation.flight_controller.read_state("gomspace.wdt_i2c_time_left")
        simulation.flight_controller.read_state("gomspace.counter_wdt_i2c")
        simulation.flight_controller.read_state("gomspace.counter_boot")

        simulation.flight_controller.read_state("gomspace.temp.output1")
        simulation.flight_controller.read_state("gomspace.temp.output2")
        simulation.flight_controller.read_state("gomspace.temp.output3")
        simulation.flight_controller.read_state("gomspace.temp.output4")

        simulation.flight_controller.read_state("gomspace.bootcause")
        simulation.flight_controller.read_state("gomspace.battmode")
        simulation.flight_controller.read_state("gomspace.pptmode")

        # writable fields

        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output1_cmd")
        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output2_cmd")
        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output3_cmd")
        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output4_cmd")
        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output5_cmd")
        simulation.flight_controller.read_state(
            "gomspace.power_cycle_output6_cmd")

        simulation.flight_controller.read_state("gomspace.pv1_cmd")
        simulation.flight_controller.read_state("gomspace.pv2_cmd")
        simulation.flight_controller.read_state("gomspace.pv3_cmd")

        simulation.flight_controller.read_state("gomspace.pptmode_cmd")

        simulation.flight_controller.read_state("gomspace.heater_cmd")

        simulation.flight_controller.read_state("gomspace.counter_reset_cmd")

        simulation.flight_controller.read_state("gomspace.gs_reset_cmd")

        simulation.flight_controller.read_state("gomspace.gs_reboot_cmd")
