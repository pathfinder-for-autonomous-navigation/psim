# Nathan Zimmerberg, Tanishq Aggarwal
# 9.14.2019
# simulation.py
# Class to run a simulation and communicate with the flight computers.

import time, timeit
import math
import threading
import matlab.engine
import datetime
import os
import json

class Simulation(object):
    """"""
    def __init__(self,devices,seed,print_log=True):
        """
        Initializes self

        Args:
            devices: Connected Teensy devices that are controllable
            seed(int or None) random number generator seed or None
            print_log: If true, prints logging messages to the console rather than
                       just to a file.
        """
        self.devices = devices
        self.flight_controller = self.devices['FlightController']
        self.truth_trajectory = []
        self.log = ""
        self.print_log = print_log

    def start(self, duration):
        '''
        Start the MATLAB simulation. This function is blocking until the simulation begins.
        '''

        self.sim_duration = duration
        self.sim_time = 0
        self.sim_thread = threading.Thread(name="Python-MATLAB Simulation Interface",
                                           target=self.run)

        self.add_to_log("Configuring simulation (please be patient)...")
        self.running = True
        self.configure_sim()
        self.sim_thread.start()

    def add_to_log(self, msg):
        if self.print_log:
            print(msg)
        self.log += f"[{datetime.datetime.now()}] {msg}\n"

    def configure_sim(self):
        start_time = timeit.default_timer()

        self.eng = matlab.engine.start_matlab()
        path1 = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "../../MATLAB")
        path2 = os.path.join(path1, "utl")
        path3 = os.path.join(path1, "environmental_models")
        path4 = os.path.join(path1, "environmental_models/helper_functions")
        self.eng.addpath(path1, nargout=0)
        self.eng.addpath(path2, nargout=0)
        self.eng.addpath(path3, nargout=0)
        self.eng.addpath(path4, nargout=0)

        self.eng.eval("global const", nargout=0)
        self.eng.config(nargout=0)

        elapsed_time = timeit.default_timer() - start_time

        self.add_to_log(
            "Configuring simulation took %0.2fs. Starting simulation loop." % elapsed_time
        )

    def run(self):
        """
        Runs the simulation for the time interval specified in start().
        """

        dt = self.eng.workspace['const']['dt'] * 1E-9
        start_time = time.time()
        while self.sim_time < self.sim_duration and self.running:
            # Update sensor model and begin update of truth model in the background

            # Send sensor data to Flight Controller, and collect actuator outputs

            # Update actuators that are not updated by flight controller

            # Actuate the sim devices, and prepare for the next cycle
            self.sim_time += dt
            time.sleep(dt - ((time.time() - start_time) % dt))

        self.running = False
        self.add_to_log("Simulation ended.")
        self.eng.quit()

    def stop(self, data_dir):
        """
        Stops a run of the simulation and saves run data to disk.
        """
        self.running = False
        self.sim_thread.join()

        with open(data_dir + "/simulation_data.txt", "w") as fp:
            json.dump(self.truth_trajectory, fp)

        with open(data_dir + "/simulation_log.txt", "w") as fp:
            fp.write(self.log)
