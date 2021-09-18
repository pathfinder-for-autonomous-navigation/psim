
from psim import Configuration, sims, Simulation

import lin
import numpy as np


configs = ['sensors/base', 'truth/base', 'truth/near_field', 'fc/base']
configs = ['config/parameters/' + f + '.txt' for f in configs]

config = Configuration(configs)
sim = Simulation(sims.OrbitControllerTest, config)

sun_ecef=sim['truth.follower.environment.s.ecef']
r=np.array(sim['truth.follower.orbit.r.ecef'])
v=np.array(sim['truth.follower.orbit.v.ecef'])
orb_plane=np.cross(r,v)/np.linalg.norm(np.cross(r,v))
proj_sun=sun_ecef-(np.dot(sun_ecef,orb_plane)*orb_plane)
theta=np.arctan2(np.linalg.norm(np.cross(proj_sun,r)),np.dot(proj_sun,r))
w=np.linalg.norm(v)/np.linalg.norm(r)
firing_nodes={np.pi/18, np.pi/6, np.pi*(5/18), np.pi*(7/18), np.pi/2, np.pi*(11/18), 
                                np.pi*(13/18), np.pi*(5/6), np.pi*(17/18), -np.pi*(17/18),
                                -np.pi*(5/6), -np.pi*(13/18), -np.pi*(11/18), -np.pi/2,
                                -np.pi*(7/18), -np.pi*(5/18), -np.pi/6, -np.pi/18}
min_time=69420

for i in firing_nodes:
    time_til_node = (i - theta) / w
    if (time_til_node > 0 and time_til_node < min_time):
        min_time = time_til_node

print(min_time)
 

