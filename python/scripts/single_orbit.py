"""Example script showing how to run a single orbit simulation.
"""

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import lin
import psim


class Simulation(psim.Simulation):

    def __init__(self):
        super(Simulation, self).__init__(psim.SingleOrbitGnc, "config/parameters/single_orbit.txt")
    
        self.ts = []
        self.rx = []
        self.ry = []
        self.rz = []

        self.n = 0

    def step(self):
        super(Simulation, self).step()

        self.n = self.n + 1
        if self.n % 100 == 0:
            r = self["truth.leader.orbit.r.eci"]
            self.rx.append(r[0])
            self.ry.append(r[1])
            self.rz.append(r[2])

            self.ts.append(self["truth.t.s"])

            self.n = 0


sim = Simulation()
steps = 3600.0 * 24.0 / sim['truth.dt.s']  # Simulate for a day
for _ in range(int(steps)):
    sim.step()

fig = plt.figure()
plt.plot(sim.rx, sim.ry, 'b.')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Position Projected into the XY Plane')
fig.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(sim.rx, sim.ry, sim.rz, 'b-')
fig.show()

plt.figure()
plt.plot(sim.ts, sim.rz, 'b.')
plt.xlabel("t (s)")
plt.ylabel("z (m)")
plt.title('Z Position')
plt.show()
