# Runs mission from startup state to standby state.
from .base import SingleSatOnlyCase, TestCaseFailure

class DeploymentToStandby(SingleSatOnlyCase):
    @property
    def run_sim(self):
        return True

    def setup_case_singlesat(self):
        self.test_stage = 'startup'
        self.deployment_length = 10 # Number of cycles for which the satellite will be in a deployment hold. This
                                    # is an item that is configured on Flight Software.
        self.max_detumble_cycles = 10 # Number of cycles for which we expect the satellite to be in detumble

    def run_case_singlesat(self):
        if self.test_stage == 'startup':
            if not self.satellite_is_in_state('startup'):
                raise TestCaseFailure("Satellite was not in state {self.test_stage}.")
            self.elapsed_deployment = 0
            self.test_stage = 'deployment_hold'

        elif self.test_stage == 'deployment_hold':
            self.elapsed_deployment += 1
            if self.elapsed_deployment == 10:
                self.test_stage = 'detumble'

        elif self.test_stage == 'detumble':
            if not self.satellite_is_in_state('detumble'):
                raise TestCaseFailure("Satellite failed to exit deployment wait period.")
            else:
                self.num_detumble_cycles = 0
                self.test_stage = 'detumble_wait'
        
        elif self.test_stage == 'detumble_wait':
            self.num_detumble_cycles += 1
            if self.num_detumble_cycles >= self.max_detumble_cycles:
                if not self.satellite_is_in_state('standby'):
                    raise TestCaseFailure("Satellite failed to exit detumble.")
                else:
                    self.test_stage = 'standby'

        elif self.test_stage == 'standby':
            print("Finished test case.")

        else:
            raise TestCaseFailure("Invalid test case stage: {self.test_stage}.")

    def satellite_is_in_state(self, state_name):
        return self.sim.flight_controller.read_state("pan.state") == str(self.mission_states[state_name])
