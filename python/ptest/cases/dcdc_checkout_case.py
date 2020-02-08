from .base import SingleSatOnlyCase

class DCDCCheckoutCase(SingleSatOnlyCase):

    def setup_case_singlesat(self, simulation):
        simulation.flight_controller.write_state(
            "pan.state", 9)  # Manual state
        self.run_case_singlesat(simulation)
        print("DCDC cases finished.")

    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state("pan.cycle_no")

        adcs_cmd = simulation.flight_controller.read_state("dcdc.ADCSMotor_cmd")
        sph_cmd = simulation.flight_controller.read_state("dcdc.SpikeDock_cmd")
        disable_cmd = simulation.flight_controller.read_state("dcdc.disable_cmd")
        reset_cmd = simulation.flight_controller.read_state("dcdc.reset_cmd")
        adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
        sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
        if adcs_cmd==sph_pin and sph_cmd==sph_pin and disable_cmd=="false" and reset_cmd=="false":
            print("Control task initialized correctly")

        # Try both DCDCs on. Turn on all systems (ADCS motors, prop valves, docking motor). 

        print ("Test case 1: ")

        simulation.flight_controller.write_state("dcdc.ADCSMotor_cmd", "true")
        simulation.flight_controller.write_state("dcdc.SpikeDock_cmd", "true")
        adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
        sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")

        if adcs_pin=="true" and sph_pin=="true":
            print("Passed")
        else:
            print("Failed")
        
        # ADCS DCDC on, SPH + Prop DCDC off. Turn on all systems. 

        print("Test case 2: ")

        simulation.flight_controller.write_state("dcdc.ADCSMotor_cmd", "true")
        simulation.flight_controller.write_state("dcdc.SpikeDock_cmd", "false")
        adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
        sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
        if adcs_pin=="true" and sph_pin=="false":
            simulation.flight_controller.write_state("dcdc.reset_cmd", "true")
            simulation.flight_controller.read_state("dcdc.ADCSMotor")
            simulation.flight_controller.read_state("dcdc.SpikeDock")
            # Reset takes at least one control cycle to complete
            adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
            sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
            if (adcs_pin == "true" and sph_pin == "true"):
                print("Passed")
            else:
                print("Unable to reset pins")
        else:
            print("Unable to turn ADCS Motor on and SpikeDock off")

        # ADCS DCDC off, SPH + Prop DCDC on. Turn on all systems. 

        print("Test Case 3: ")

        simulation.flight_controller.write_state("dcdc.ADCSMotor_cmd", "false")
        simulation.flight_controller.write_state("dcdc.SpikeDock_cmd", "true")
        adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
        sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
        if adcs_pin=="false" and sph_pin=="true":
            simulation.flight_controller.write_state("dcdc.reset_cmd", "true")
            simulation.flight_controller.read_state("dcdc.ADCSMotor")
            simulation.flight_controller.read_state("dcdc.SpikeDock")
            # Reset takes at least one control cycle to complete
            adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
            sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
            if (adcs_pin == "true" and sph_pin == "true"):
                print("Passed")
            else:
                print("Unable to reset pins")
        else:
            print("Unable to turn ADCS Motor off and SpikeDock on")

        # ADCS DCDC off, SPH + Prop DCDC off. Turn on all systems. 

        print("Test Case 4: ")

        simulation.flight_controller.write_state("dcdc.disable_cmd", "true")
        adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
        sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
        if adcs_pin=="false" and sph_pin=="false":
            simulation.flight_controller.write_state("dcdc.reset_cmd", "true")
            simulation.flight_controller.read_state("dcdc.ADCSMotor")
            simulation.flight_controller.read_state("dcdc.SpikeDock")
            # Reset takes at least one control cycle to complete
            adcs_pin = simulation.flight_controller.read_state("dcdc.ADCSMotor")
            sph_pin = simulation.flight_controller.read_state("dcdc.SpikeDock")
            if (adcs_pin == "true" and sph_pin == "true"):
                print("Passed")
            else:
                print("Unable to reset pins")
        else:
            print("Unable to disable pins")