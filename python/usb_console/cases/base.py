# Base classes for writing testcases.

class TestCaseFailure(Exception):
    """Raise in case of test case failure."""

class Case(object):
    @property
    def run_sim(self):
        return False

    @property
    def single_sat_compatible(self):
        raise NotImplementedError

    def setup_case(self, simulation):
        self.sim = simulation
        self._setup_case()

    def _setup_case(self):
        raise NotImplementedError

    def run_case(self):
        raise NotImplementedError

# Base testcase for writing testcases that only work with a single-satellite mission.
class SingleSatOnlyCase(Case):
    @property
    def single_sat_compatible(self):
        return True

    def run_case(self):
        if not self.sim.is_single_sat_sim:
            raise Exception(f"Testcase {__class__.__name__} only works for a single-satellite simulation.")
        self.run_case_singlesat()

    def run_case_singlesat(self):
        raise NotImplementedError

# Base testcase for writing testcases that only work with a full mission simulation
# with both satellites.
class MissionCase(Case):
    @property
    def single_sat_compatible(self):
        return False

    def run_case(self):
        if self.sim.is_single_sat_sim:
            raise Exception(f"Testcase {__class__.__name__} only works for a full-mission simulation.")
        self.run_case_fullmission()

    def run_case_fullmission(self):
        raise NotImplementedError

class FlexibleCase(Case):
    @property
    def single_sat_compatible(self):
        return True

    def _setup_case(self):
        if self.sim.is_single_sat_sim:
            self.setup_case_singlesat()
        else:
            self.setup_case_fullmission()

    def setup_case_singlesat(self):
        raise NotImplementedError

    def setup_case_fullmission(self):
        raise NotImplementedError

    def run_case(self):
        if self.sim.is_single_sat_sim:
            self.run_case_singlesat()
        else:
            self.run_case_fullmission()

    def run_case_singlesat(self):
        raise NotImplementedError

    def run_case_fullmission(self):
        raise NotImplementedError
