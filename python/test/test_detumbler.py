from psim import Configuration, sims, Simulation

import lin
import pytest


def test_detumbler():
    """Test the detumbler.

    This boots the simulation starting right after the deployment hold has been
    completed - i.e. forty minutes after a simultaneous deployment. We're
    essentially checking that the detumbler can bring the total angular momentum
    of the spacecraft below a given threshold in a appropriate amount of time.

    This currently detumbles in about 22361 steps.
    """
    configs = ['sensors/base', 'truth/base', 'truth/detumble']
    configs = ['config/parameters/' + f + '.txt' for f in configs]

    config = Configuration(configs)
    sim = Simulation(sims.DetumblerTest, config)

    # Wait up to four hours for the spacecraft to have detumbled
    timeout = 4 * 60 * 60 * 1000000000 + sim['truth.t.ns']

    # Angular momentum threshold to determine if we've detumbled
    #
    # Reference(s):
    #  - https://github.com/pathfinder-for-autonomous-navigation/FlightSoftware/blob/2e3e133c49c44e8c792f3c7bef1b6e43ad2f3141/src/fsw/FCCode/MissionManager.cpp#L201-L215
    threshold = 1047 * 1.35e-5 * 0.33

    sim.step()
    while sim['truth.leader.attitude.L.norm'] > threshold:
        assert sim['truth.t.ns'] < timeout, 'Spacecraft failed to detumble in alloted time'
        sim.step()
