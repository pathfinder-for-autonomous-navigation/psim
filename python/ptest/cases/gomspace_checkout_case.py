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

        def read_state(self, string_state):
            return simulation.flight_controller.read_state(string_state)

        def write_state(self, string_state, state_value):
            simulation.flight_controller.write_state(string_state, state_value)
            return read_state(self, string_state)

        # readable fields

        vboost = [read_state(self, "gomspace.vboost.output" + str(i))
                  for i in range(1, 4)]

        vbatt = simulation.flight_controller.read_state("gomspace.vbatt")
        if vbatt < 6000 or vbatt > 8400:
            print(
                "Vbatt is out of expected range [6000, 8400] mV at: " + str(vbatt))

        curin = [read_state(self, "gomspace.curin.output" + str(i))
                 for i in range(1, 4)]

        cursun = read_state(self, "gomspace.cursun")
        cursys = read_state(self, "gomspace.cursys")

        curout = [read_state(self, "gomspace.curout.output" + str(i))
                  for i in range(1, 7)]

        output = [read_state(self, "gomspace.output.output" + str(i))
                  for i in range(1, 7)]

        for n in range(0, len(output)):
            out_n = output[n]
            # TODO: check umbilical for which outputs should be 0 and 1
            if out_n < 0.5 or out_n > 3.0:
                print("Output-" + n +
                      " is out of range [0.3, 0.5] at: " + out_n)

        wdt_i2c_time_left = simulation.flight_controller.read_state(
            "gomspace.wdt_i2c_time_left")
        if wdt_i2c_time_left > 99:
            print("wdt_i2c_time_left is greater than 99 seconds at: " +
                  str(wdt_i2c_time_left))

        counter_wdt_i2c = simulation.flight_controller.read_state(
            "gomspace.counter_wdt_i2c")
        print("counter_wdt_i2c is: " + str(counter_wdt_i2c))
        counter_boot = simulation.flight_controller.read_state(
            "gomspace.counter_boot")
        print("counter_wdt_i2c is: " + str(counter_boot))

        temp = [read_state(self, "gomspace.temp.output" + str(i))
                for i in range(1, 5)]
        for n in range(0, len(temp)):
            temp_n = temp[n]
            if temp_n < 20 or out_n > 22:
                print("Temp-" + n +
                      " is out of room temperature range [20, 22] degC at: " + str(temp_n))

        bootcause = read_state(self, "gomspace.bootcause")
        print("bootcause is: " + str(bootcause))
        battmode = read_state(self, "gomspace.battmode")
        print("battmode is: " + str(battmode))
        pptmode = read_state(self, "gomspace.pptmode")
        print("pptmode is: " + str(pptmode))

        # writable fields
        power_cycle_output_cmd = [read_state(self, "gomspace.power_cycle_output" + str(i) + "_cmd")
                                  for i in range(1, 7)]
        power_cycle_output_cmd_updated = [write_state(self, "gomspace.power_cycle_output" + str(i) + "_cmd",
                                                      not power_cycle_output_cmd[i])
                                          for i in range(0, len(power_cycle_output_cmd))]
        for n in range(0, len(power_cycle_output_cmd)):
            if power_cycle_output_cmd[n] == power_cycle_output_cmd_updated[n]:
                print("Could not update power_cycle_output" + str(n))

        ppt_mode_cmd = read_state(self, "gomspace.pptmode_cmd")

        heater_cmd = read_state(self, "gomspace.heater_cmd")
        heater_cmd_updated = write_state(
            self, "gomspace.heater_cmd", not heater_cmd)
        if heater_cmd == heater_cmd_updated:
            print("Could not update heater")

        counter_reset_cmd = read_state(self, "gomspace.counter_reset_cmd")
        counter_reset_cmd_updated = write_state(
            self, "gomspace_reset_cmd", not counter_reset_cmd)
        if counter_reset_cmd == counter_reset_cmd_updated:
            print("Could not update gomspace_reset")

        gs_reset_cmd = read_state(self, "gomspace.gs_reset_cmd")
        gs_reset_cmd_updated = write_state(
            self, "gomspace.gs_reset_cmd", not gs_reset_cmd)
        if gs_reset_cmd == gs_reset_cmd_updated:
            print("Could not update gs_reset")

        gs_reboot_cmd = read_state(self, "gomspace.gs_reboot_cmd")
        gs_reboot_cmd_updated = write_state(
            self, "gomspace.gs_reboot_cmd", not gs_reset_cmd)
        if gs_reboot_cmd == gs_reboot_cmd_updated:
            print("Could not update gs_reboot")
