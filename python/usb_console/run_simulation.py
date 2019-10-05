#!/usr/local/bin/python3

from argparse import ArgumentParser
from state_session import StateSession
from cmdprompt import StateCmdPrompt
from data_containers import Logger, Datastore
from simulation import Simulation
import json, sys, os, threading, time

class SimulationRun(object):
    def __init__(self, random_seed, data_dir, device_data):
        self.random_seed = random_seed
        self.data_dir = data_dir
        self.device_data = device_data

        self.datastores = {}
        self.loggers = {}
        self.devices = {}

    def start(self):
        if not device_data:
            print('Error: must specify at least one serial port.')
            raise SystemExit

        # Set up test table by connecting to each device specified in the config.
        for device in self.device_data:
            device_name = device["name"]

            simulation_run_dir = os.path.join(os.path.abspath(self.data_dir), time.strftime("%Y%m%d-%H%M%S"))
            os.makedirs(simulation_run_dir, exist_ok=True)
            device_datastore = Datastore(device_name, simulation_run_dir)
            device_logger = Logger(device_name, simulation_run_dir)

            port_cmd = StateSession(self.data_dir, device_name, device_datastore, device_logger)
            if port_cmd.connect(device["port"], 115200):
                self.devices[device_name] = port_cmd
                self.datastores[device_name] = device_datastore
                self.loggers[device_name] = device_logger

                device_datastore.start()
                device_logger.start()
            elif device["stop_if_unconnected"]:
                print("Error: exiting due to a required serial port being disconnected.")
                self.stop_all()
                return

        # Set up MATLAB simulation
        sim = Simulation(self.devices, self.random_seed)
        self.sim_thread = threading.Thread(name="Python-MATLAB Simulation Interface", target=sim.run, args=[2])
        self.sim_thread.start()

        # User command prompt
        cmd_prompt = StateCmdPrompt(self.devices, self.stop_all)
        pan_logo_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pan_logo.txt')
        with open(pan_logo_filepath, 'r') as pan_logo_file:
            cmd_prompt.intro = pan_logo_file.read()
        cmd_prompt.prompt = '> '
        try:
            cmd_prompt.cmdloop()
        except KeyboardInterrupt:
            # Gracefully exit session
            cmd_prompt.do_quit(None)
            self.stop_all()

    def stop_all(self):
        self.sim_thread.join()

        print("Stopping loggers...")
        for datastore in self.datastores.values():
            datastore.stop()
        for logger in self.loggers.values():
            logger.stop()

        print("Terminating device connections...")
        for device in self.devices.values():
            device.disconnect()

        raise SystemExit

if __name__ == '__main__':
    if sys.version_info[0] != 3 or sys.version_info[1] < 6:
        print("Running this script requires Python 3.6 or above.")
        sys.exit(1)

    parser = ArgumentParser(description='''
    Interactive console allows sending state commands to PAN Teensy devices, and parses console output 
    from Teensies into human-readable, storable logging information.''')

    parser.add_argument('-c', '--conf', action='store', help='JSON file listing serial ports and Teensy computer names. Default is config.json.',
                        default = 'config.json')
    parser.add_argument('-d', '--data-dir', action='store',
        help='''Directory for storing run data, relative to the location of the console script. Default is logs/.
                For the current run, a subdirectory of DATA_DIR is created in which the actual data is stored.''', default='logs')
    args = parser.parse_args()

    try:
        with open(args.conf, 'r') as config_file:
            config_data = json.load(config_file)
            device_data = config_data["devices"]
            random_seed = config_data["seed"]
    except json.JSONDecodeError:
        print("Could not load config file. Exiting.")
        raise SystemExit
    except KeyError:
        print("Malformed config file. Exiting.")
        raise SystemExit

    simulation_run = SimulationRun(random_seed, args.data_dir, device_data)
    simulation_run.start()
