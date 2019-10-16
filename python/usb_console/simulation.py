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
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "../../MATLAB")
        self.eng.addpath(path, nargout=0)

        self.eng.config(nargout=0)
        self.eng.eval("global const", nargout=0)
        self.eng.eval("global main_state_trajectory", nargout=0)
        self.eng.eval("global computer_state_follower_trajectory", nargout=0)
        self.eng.eval("global computer_state_leader_trajectory", nargout=0)

        self.main_state = self.eng.initialize_main_state(1,'not_detumbled', nargout=1)
        self.computer_state_follower, self.computer_state_leader = self.eng.initialize_computer_states('not_detumbled', nargout=2)
        self.main_state_trajectory = []
        self.computer_state_follower_trajectory = []
        self.computer_state_leader_trajectory = []

        elapsed_time = timeit.default_timer() - start_time

        self.add_to_log(
            "Configuring simulation took %0.2fs. Starting simulation loop." % elapsed_time
        )

    def run(self):
        """
        Runs the simulation for the time interval specified in start().
        """


        dt = self.eng.workspace['const']['dt'] * 1E-9
        num_steps = int(self.sim_duration/dt)
        sample_rate = 10.0 / dt # Sample once every ten seconds
        step = 0

        start_time = time.time()
        while step < num_steps and self.running:
            self.sim_time = self.main_state['follower']['dynamics']['time']

            # Get sensor readings
            self.sensor_readings_follower = self.eng.sensor_reading(self.main_state['follower'],self.main_state['leader'], nargout=1)
            self.sensor_readings_leader = self.eng.sensor_reading(self.main_state['leader'],self.main_state['follower'], nargout=1)
            # update dynamics
            self.main_state = self.eng.main_state_update(self.main_state, nargout=1)
            # simulate flight computers
            self.computer_state_follower, self.actuator_commands_follower = \
                self.eng.update_FC_state(self.computer_state_follower,self.sensor_readings_follower, nargout=2)
            self.computer_state_leader, self.actuator_commands_leader = \
                self.eng.update_FC_state(self.computer_state_leader,self.sensor_readings_leader, nargout=2)
            # command actuators
            self.main_state['follower'] = self.eng.actuator_command(self.actuator_commands_follower,self.main_state['follower'], nargout=1)
            self.main_state['leader'] = self.eng.actuator_command(self.actuator_commands_leader,self.main_state['leader'], nargout=1)

            # store trajectory
            if step % sample_rate == 0:
                self.main_state_trajectory.append(
                    json.loads(self.eng.jsonencode(self.main_state, nargout=1)))
                self.computer_state_follower_trajectory.append(
                    json.loads(self.eng.jsonencode(self.computer_state_follower, nargout=1)))
                self.computer_state_leader_trajectory.append(
                    json.loads(self.eng.jsonencode(self.computer_state_leader, nargout=1)))

            step += 1
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

        with open(data_dir + "/simulation_data_main.txt", "w") as fp:
            json.dump(self.main_state_trajectory, fp)

        with open(data_dir + "/simulation_data_follower.txt", "w") as fp:
            json.dump(self.computer_state_follower_trajectory, fp)

        with open(data_dir + "/simulation_data_leader.txt", "w") as fp:
            json.dump(self.computer_state_leader_trajectory, fp)

        with open(data_dir + "/simulation_log.txt", "w") as fp:
            fp.write(self.log)
