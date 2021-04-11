
from psim import Configuration, sims, Simulation

import lin
import pytest
import numpy as np

def test_orbit_controller():
    """Test the orbit controller.
    This boots the simulation starting in standby (after detumbling)
    """
    configs = ['sensors/base', 'truth/base', 'truth/standby']
    configs = ['config/parameters/' + f + '.txt' for f in configs]

    config = Configuration(configs)
    sim = Simulation(sims.OrbitControllerTest, config)

    sun_ecef=sim['truth.follower.environment.s.ecef']
    r=np.array(sim['truth.follower.orbit.r.ecef'],copy=False)
    v=np.array(sim['truth.follower.orbit.v.ecef'],copy=False)
    orb_plane=np.cross(r,v)/np.linalg.norm(np.cross(r,v))
    proj_sun=sun_ecef-(np.dot(sun_ecef,orb_plane)*orb_plane)
    theta=np.arctan2(np.linalg.norm(np.cross(proj_sun,r)),np.dot(proj_sun,r))
    w=np.linalg.norm(v)/np.linalg.norm(r)
    firing_nodes={-np.pi/3,np.pi,np.pi/3}
    min_time=69420

    for i in firing_nodes:
        time_til_node = (i - theta) / w
        if (time_til_node > 0 and time_til_node < min_time):
            min_time = time_til_node

    assert 1==2 , min_time
 