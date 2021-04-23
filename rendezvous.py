from psim import Configuration, Simulation
from psim.sims import OrbitMpcRendezvous

import cvxpy as cp
import numpy as np
import scipy as sp

import lin


# Number of steps taken between manuevers
T = 1765

# Prediction horizon
N = 100

# Base configuration for simulations
configs = ['sensors/base', 'truth/base', 'truth/near_field']
configs = ['config/parameters/' + config + '.txt' for config in configs]
config = Configuration(configs)

# Fields being logged to CSV from the simulation
fields = [
    'truth.t.ns',
    'truth.follower.hill.dr', 'truth.follower.hill.dv',
    'fc.follower.relative_orbit.is_valid',
    'fc.follower.relative_orbit.r.hill', 'fc.follower.relative_orbit.v.hill'
]

# Output CSV file
filename = 'logs.csv'

# Final stage cost
QT = sp.sparse.diags([1e4, 1e4, 1e4, 1e4, 1e4, 1e4])

# Stage costs
Q = sp.sparse.diags([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
R = sp.sparse.diags([1.0, 1.0, 1.0])

# Maximum allowable impulse
J_max = 0.025

# Satellite mass
m = 3.6


class RendezvousController(object):
    """Implements a model-predictive control algorithm for near-field
    orbital rendezvous.
    """
    def __init__(self, N, QT, Q, R, J_max):
        super(RendezvousController, self).__init__()

        # System dynamics
        self.A = cp.Parameter((6,6), name='A')
        self.B = cp.Parameter((6,3), name='B')

        # Initial state
        self.x = cp.Parameter((6), name='x')

        # Decision Variables
        self.X = cp.Variable((6,N+1), name='X')
        self.U = cp.Variable((3,N), name='U')

        # Objective
        objective = cp.quad_form(self.X[:,N], QT)
        for i in range(N):
            objective += \
                    cp.quad_form(self.X[:,i], Q) + cp.quad_form(self.U[:,i], R)

        # Constraints
        constraints = [self.X[:,0] == self.x]
        for i in range(N):
            constraints += [
                self.X[:,i+1] == self.A @ self.X[:,i] + self.B @ self.U[:,i],
                cp.norm(self.U[:,i]) <= J_max
            ]

        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    @staticmethod
    def cw(dt, n):
        """Generates the Clohessy-Wilshire discrete-time dynamics.
        """
        nt = n*dt
        snt = np.sin(nt)
        cnt = np.cos(nt)

        return np.array([
            [      4.0 - 3.0 * cnt, 0.0,      0.0,               snt / n,      2.0 * (1.0 - cnt) / n,     0.0],
            [     6.0 * (snt - nt), 1.0,      0.0, 2.0 * (cnt - 1.0) / n, (4.0 * snt - 3.0 * nt) / n,     0.0],
            [                  0.0, 0.0,      cnt,                   0.0,                        0.0, snt / n],
            [        3.0 * n * snt, 0.0,      0.0,                   cnt,                  2.0 * snt,     0.0],
            [6.0 * n * (cnt - 1.0), 0.0,      0.0,            -2.0 * snt,            4.0 * cnt - 3.0,     0.0],
            [                  0.0, 0.0, -n * snt,                   0.0,                        0.0,     cnt]
        ])

    def run(self, dt, n, m, dr, dv):
        """Calculates desired impulse at the current timestep.
        """
        import matplotlib.pyplot as plt

        # Calculate our dynamics
        self.A.value = RendezvousController.cw(dt, n)
        self.B.value = self.A.value[:,3:6] / m

        # Initial state
        self.x.value = np.array([dr[0], dr[1], dr[2], dv[0], dv[1], dv[2]])

        # Solve
        self.problem.solve()
        if self.problem.status != cp.OPTIMAL:
            print('Solver status did not report as optimal!')

        # print(self.problem.status)
        # print(self.problem.value)
        # print(self.U.value)
        # print(self.A.value)
        # print(self.B.value)
        # print(dt)
        # print(n)

        # fg = plt.figure()
        # ax = fg.add_subplot(111, projection='3d')
        # ax.plot(self.X.value[0,:], self.X.value[1,:], self.X.value[2,:])
        # fg.show()

        # fg = plt.figure()
        # plt.plot(self.X.value[0,:], label='r.x')
        # plt.plot(self.X.value[1,:], label='r.y')
        # plt.plot(self.X.value[2,:], label='r.z')
        # plt.legend()
        # fg.show()

        # fg = plt.figure()
        # plt.plot(self.X.value[3,:], label='v.x')
        # plt.plot(self.X.value[4,:], label='v.y')
        # plt.plot(self.X.value[5,:], label='v.z')
        # plt.legend()
        # fg.show()

        # plt.figure()
        # plt.plot(self.U.value[0,:], label='u.x')
        # plt.plot(self.U.value[1,:], label='u.y')
        # plt.plot(self.U.value[2,:], label='u.z')
        # plt.legend()
        # plt.show()

        # assert False

        return lin.Vector3(self.U.value[:,0])


# Initialize logging arrays
logs = {
    'fc.follower.orbit.J.hill': list(),
    'fc.follower.orbit.J.hill.norm': list()
}
for field in fields:
    logs[field] = list()

# Construct the simulation
config['sensors.follower.cdgps.model_range'] = False
sim = Simulation(OrbitMpcRendezvous, config)
dt = float(sim['truth.dt.ns'] * T) / 1e9

# Construct the solver
controller = RendezvousController(N, QT, Q, R, J_max)

sim.step()
sim.step()

# Main simulation loop
for _ in range(150):

    # Mean motion of the follower
    r = sim['truth.follower.orbit.r.eci']
    v = sim['truth.follower.orbit.v.eci']
    n = lin.norm(lin.cross(r, v) / lin.fro(r))

    # Run the controller
    dr = sim['fc.follower.relative_orbit.r.hill']
    dv = sim['fc.follower.relative_orbit.v.hill']

    if not sim['sensors.follower.cdgps.valid']:
        print(f'Lost CDGPS; ||dr|| = {lin.norm(dr)}')

    if lin.norm(dr) < 0.20:
        print('Rendezvous complete')
        break

    u = controller.run(dt, n, m, dr, dv)
    sim['fc.follower.orbit.J.hill'] = u

    logs['fc.follower.orbit.J.hill'].append(u)
    logs['fc.follower.orbit.J.hill.norm'].append(lin.norm(u))
    for field in fields:
        logs[field].append(sim[field])

    sim.step()

    for _ in range(T-1):
        logs['fc.follower.orbit.J.hill'].append(lin.Vector3())
        logs['fc.follower.orbit.J.hill.norm'].append(0)
        for field in fields:
            logs[field].append(sim[field])

        sim.step()

# Save data to a CSV
with open(filename, 'w') as csv:
    keys = sorted(logs.keys())
    suffixes = ['x', 'y', 'z', 'w']

    for key in keys:
        value = logs[key][0]
        if type(value) in [lin.Vector2, lin.Vector3, lin.Vector4]:
            for j in range(value.size()):
                csv.write(f'{key}.{suffixes[j]},')
        else:
            csv.write(f'{key},')
    csv.write('\n')

    for i in range(len(logs['truth.t.ns'])):
        for key in keys:
            value = logs[key][i]
            if type(value) in [lin.Vector2, lin.Vector3, lin.Vector4]:
                for j in range(value.size()):
                    csv.write(f'{value[j]},')
            else:
                csv.write(f'{value},')
        csv.write('\n')
