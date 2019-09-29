#Nathan Zimmerberg
#9.14.2019
#simulation.py
#Class to run a simulation and communicate with the flight computers.
import time
import math
class Simulation(object):
    """"""
    def __init__(self,state_session_fc,seed):
        """Initializes self

        Args:
            state_session_fc(a connected StateSession): flight controller
            seed(int or None) random number generator seed or None"""
        self.flight_controller=state_session_fc
    def start(self,time):
        """runs the simulation for time seconds

        Args:
            time(float) length of simulation"""
        startns=time.clock_gettime_ns()
        endns=int(startns+time*1E9)
        dtns=int(1E8)
        n= int(time/0.1)
        for i in range(n):
            gps_pos=[0,1,2]
            self.flight_controller.write_state_fb('gps_pos',gps_pos)
            while(time.clock_gettime_ns()-startns <dtns*i)
