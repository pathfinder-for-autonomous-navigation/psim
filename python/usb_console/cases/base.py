# Base classes for writing testcases.

class SimCase(object):
    def run_case(self, simulation):
        raise NotImplementedError

class SingleSatOnlySimCase(SimCase):
    single_sat_sim_compatible = True

    def run_case(self, simulation):
        if not simulation.is_single_sat_sim:
            raise Exception(f"Testcase {__class__.__name__} only works for a single-satellite simulation.")
        self.run_case_singlesat(simulation)

    def run_case_singlesat(self, simulation):
        raise NotImplementedError

class MissionSimCase(SimCase):
    single_sat_sim_compatible = False

    def run_case(self, simulation):
        if simulation.is_single_sat_sim:
            raise Exception(f"Testcase {__class__.__name__} only works for a full-mission simulation.")
        self.run_case_fullmission(simulation)

    def run_case_fullmission(self, simulation):
        raise NotImplementedError

class FlexibleSimCase(SimCase):
    single_sat_sim_compatible = True

    def run_case(self, simulation):
        if simulation.is_single_sat_sim:
            self.run_case_singlesat(simulation)
        else:
            self.run_case_fullmission(simulation)

    def run_case_singlesat(self, simulation):
        raise NotImplementedError

    def run_case_fullmission(self, simulation):
        raise NotImplementedError
