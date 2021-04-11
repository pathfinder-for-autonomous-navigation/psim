from psim import Configuration, sims, Simulation

import lin
import pytest


def test_orbit_controller():
    """Test the orbit controller.
    This boots the simulation starting in standby (after detumbling)
    """
    configs = ['sensors/base', 'truth/base', 'fc/base', 'truth/standby']
    configs = ['config/parameters/' + f + '.txt' for f in configs]

    config = Configuration(configs)
    sim = Simulation(sims.OrbitControllerTest, config)

    #The spacecrafts are given 10 days to rendezvous
    timeout = 10 * 24 * 3600 * 1e9
    threshold = 0.5
    sim.step()

    while sim['truth.leader.hill.dr.norm'] > threshold:
        assert sim['truth.t.ns'] < timeout , 'Spacecrafts failed to rendezvous in alloted time'
        sim.step()