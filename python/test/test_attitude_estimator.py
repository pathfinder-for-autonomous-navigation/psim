from psim import Configuration, sims, Simulation

import lin
import numpy as np
import pytest


def test_attitude_estimator():
    """Tests the attitude estimator.
    
    The ability of the attitude estimator to initialize itself and remain valid
    are tested. The estimated sigma bounds are also verified to be accurate -
    i.e. ninety five percent of the errors fall within the reported two sigma
    bounds.
    """
    configs = ['sensors/base', 'truth/base', 'truth/deployment']
    configs = ['config/parameters/' + f + '.txt' for f in configs]

    config = Configuration(configs)
    sim = Simulation(sims.AttitudeEstimatorTestGnc, config)

    # Wait for the attitude estimator to be initialized
    timeout = 10000
    while sim['fc.leader.attitude.is_valid'] == 0:
        assert timeout > 0
        timeout = timeout - 1
        sim.step()

    # Total number of sample points and times outside the two sigma bounds
    N = 0
    p_miss = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    g_miss = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    # Log data for two hours
    t_ns = sim['truth.t.ns']
    while sim['truth.t.ns'] - t_ns < 2 * 3600 * 1000000000:
        assert sim['fc.leader.attitude.is_valid'] != 0

        g_error = sim['fc.leader.attitude.w.bias.error']
        g_sigma = sim['fc.leader.attitude.w.bias.sigma']
        p_error = sim['fc.leader.attitude.p.body_eci.error']
        p_sigma = sim['fc.leader.attitude.p.body_eci.sigma']
        for i in range(3):
            for j in range(2):
                g_miss[i][j] = g_miss[i][j] + (abs(g_error[i]) > (j + 2) * g_sigma[i])
                p_miss[i][j] = p_miss[i][j] + (abs(p_error[i]) > (j + 2) * p_sigma[i])

        N = N + 1
        sim.step()

    # Ensure the error in our outputs resemble gaussian noise.
    #
    # Reference(s):
    #  - https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
    bounds = [1.0 - 0.9545, 1.0 - 0.9973]
    for i in range(3):
        for j in range(2):
            g_percent = g_miss[i][j] / N
            p_percent = p_miss[i][j] / N

            assert g_percent <= bounds[j]
            assert p_percent <= bounds[j]
