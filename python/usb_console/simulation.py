# Nathan Zimmerberg, Tanishq Aggarwal
# 9.14.2019
# simulation.py
# Class to run a simulation and communicate with the flight computers.

import time, timeit
import math
import threading
import matlab.engine
import os
import json

class Simulation(object):
    """"""
    def __init__(self,devices,seed):
        """
        Initializes self

        Args:
            state_session_fc(a connected StateSession): flight controller
            seed(int or None) random number generator seed or None
        """
        self.devices = devices
        self.flight_controller = self.devices['FlightController']
        self.truth_trajectory = []

    def start(self, duration):
        self.sim_thread = threading.Thread(name="Python-MATLAB Simulation Interface",
                                           target=self.run,
                                           args=[duration])
        self.sim_thread.start()

    def run(self,duration):
        """
        Runs the simulation for time seconds

        Args:
            duration(float) length of simulation
        """

        print("Configuration simulation...")

        start_time = timeit.default_timer()

        eng = matlab.engine.start_matlab()
        path1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../MATLAB")
        path2 = os.path.join(path1, "utl")
        path3 = os.path.join(path1, "environmental_models")
        path4 = os.path.join(path1, "environmental_models/helper_functions")
        eng.addpath(path1, nargout=0)
        eng.addpath(path2, nargout=0)
        eng.addpath(path3, nargout=0)
        eng.addpath(path4, nargout=0)

        eng.eval("global truth", nargout=0)
        eng.eval("global const", nargout=0)
        eng.eval("global actuators", nargout=0)
        eng.eval("global truth", nargout=0)
        eng.eval("global const", nargout=0)
        eng.eval("global truth_trajectory", nargout=0)
        eng.eval("global actuators", nargout=0)
        eng.eval("global actuator_commands", nargout=0)
        eng.eval("global sensor_readings", nargout=0)
        eng.eval("global sensor_state", nargout=0)
        eng.eval("global computer_state", nargout=0)
        eng.config(nargout=0)

        elapsed_time = timeit.default_timer() - start_time

        print(f"Configuring simulation took {elapsed_time} s. Starting simulation loop. ")

        dt = eng.workspace['const']['dt']

        while eng.workspace['truth']['mission_time'] < duration * 1E9:
            # Update sensor model and begin update of truth model in the background
            eng.workspace['sensor_readings'] = eng.sensor_reading(eng.workspace['sensor_state'],
                                                eng.workspace['truth'],
                                                eng.workspace['actuators'])

            truth = eng.orbit_attitude_update_ode2(eng.workspace['truth'], eng.workspace['actuators'], dt * 1E9, background=True)
            sensor_state_update = eng.sensor_state_update(eng.workspace['sensor_state'], eng.workspace['truth'], dt * 1E9, background=True)

            # Send sensor data to Flight Controller, and collect actuator outputs
            # self.flight_controller.write_state("prop.temp_outer", 2)
            # foo = self.flight_controller.read_state("prop.temp_inner")

            # Update actuators that are not updated by flight controller
            fc_outputs = eng.update_FC_state(eng.workspace['computer_state'], eng.workspace['sensor_readings'])
            eng.workspace['computer_state'] = fc_outputs[0]
            eng.workspace['actuator_commands'] = fc_outputs[1]

            # Wait for simulation to finish computing its truth
            eng.workspace['truth'] = truth.result()
            sensor_state_update.result()

            # Actuate the sim devices, and prepare for the next cycle
            eng.workspace['actuator'] = eng.actuator_command(eng.workspace['actuator_commands'], eng.workspace['truth'])
            eng.workspace['truth']['mission_time'] += dt * 1E9
            self.truth_trajectory.append(eng.workspace['truth'])

            time.sleep(dt * 1E9)

        print("Simulation ended.")
        # eng.quit()

    def stop(self, data_dir):
        """
        Stops a run of the simulation.
        """
        self.running = False
        self.sim_thread.join()

        with open(data_dir + "/simulation_data.txt", "w") as fp:
            json.dump(self.truth_trajectory, fp)
