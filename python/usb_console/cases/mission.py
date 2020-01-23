# Runs mission from startup state to docked state. Can be used on
# one or both satellites.
from .base import FlexibleCase

# This test case should be run without the MATLAB simulation.
class MissionStateMachineCheckout(FlexibleCase):
    mission_states = {
        "startup" : 0,
        "detumble" : 1,
        "initialization_hold" : 2,
        "follower" : 3,
        "standby" : 4,
        "leader" : 5,
        "docking" : 6,
        "docked" : 7,
        "safehold" : 8,
        "manual" : 9
    }

    def setup_case_singlesat(self, simulation):
        self.test_stage = 'startup'

    def setup_case_fullmission(self, simulation):
        self.test_stage_follower = 'startup'
        self.test_stage_leader = 'startup'

    def run_case_singlesat(self, simulation):
        if self.test_stage == 'startup':
            deployment_length = 10
            for cc in range(deployment_length):
                self._step_cycle()

            if not self._is_state('detumble'):
                raise TestCaseFailure("Satellite failed to exit deployment wait period.")
            else:
                print("Succeeded")

    def run_case_fullmission(self, simulation):
        pass

    def _step_cycle(self):
        self.sim.flight_controller.write_state("cycle.start", "true")

    def _is_state(self, state_name):
        return self.sim.flight_controller.read_state("pan.state") == str(mission_states[state_name])
