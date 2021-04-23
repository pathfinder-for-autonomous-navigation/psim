import matplotlib.pyplot as plt

import numpy as np
import pandas


data = pandas.read_csv('logs.csv')

fg = plt.figure()
ax = fg.add_subplot(111, projection='3d')
ax.plot(data['truth.follower.hill.dr.x'], data['truth.follower.hill.dr.y'],
        data['truth.follower.hill.dr.z'])
ax.set_xlabel('dr.x')
ax.set_ylabel('dr.y')
ax.set_zlabel('dr.z')
fg.show()

fg = plt.figure()
plt.plot(data['truth.follower.hill.dr.x'], label='dr.x')
plt.plot(data['truth.follower.hill.dr.y'], label='dr.y')
plt.plot(data['truth.follower.hill.dr.z'], label='dr.z')
plt.legend()
fg.show()

fg = plt.figure()
plt.plot(data['truth.follower.hill.dv.x'], label='dv.x')
plt.plot(data['truth.follower.hill.dv.y'], label='dv.y')
plt.plot(data['truth.follower.hill.dv.z'], label='dv.z')
plt.legend()
fg.show()

plt.figure()
plt.plot(data['fc.follower.orbit.J.hill.x'], label='J.x')
plt.plot(data['fc.follower.orbit.J.hill.y'], label='J.y')
plt.plot(data['fc.follower.orbit.J.hill.z'], label='J.z')
plt.legend()
plt.show()
