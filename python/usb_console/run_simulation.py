#!/usr/local/bin/python3

from argparse import ArgumentParser
from state_session import StateSession
from cmdprompt import StateCmdPrompt
from data_containers import Logger, Datastore
from simulation import Simulation
import json, sys, os, tempfile, threading, time, subprocess, pty
import urllib.request

class SimulationRun(object):
    def __init__(self, random_seed, sim_duration, data_dir, device_config):
        self.random_seed = random_seed
        self.sim_duration = sim_duration
        
        self.simulation_run_dir = os.path.join(data_dir, time.strftime("%Y%m%d-%H%M%S"))
        # Create directory for run data
        os.makedirs(self.simulation_run_dir, exist_ok=True)
        
        self.device_config = device_config

        self.datastores = {}
        self.loggers = {}
        self.devices = {}
        self.binaries = []

    def start(self):
        """Starts a run of the simulation."""

        pan_logo_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pan_logo.txt')
        with open(pan_logo_filepath, 'r') as pan_logo_file:
            print(pan_logo_file.read())

        if not self.device_config:
            print('Error: must specify at least one serial port.')
            raise SystemExit

        # Set up test table by connecting to each device specified in the config.
        for device in self.device_config:
            device_name = device["name"]

            # Check device configuration.
            if 'required' not in device.keys():
                self.stop_all(f"Error: device configuration for {device_name} does not specify if the simulation requires the device.")

            if device.get('run_mode') not in ['teensy', 'native']:
                self.stop_all(f"Error: device configuration for {device_name} is invalid.")
            if device['run_mode'] == "teensy":
                if not device.get("baud_rate"):
                    self.stop_all(f"Error: device configuration for {device_name} does not specify baud rate.")
            elif "binary_filepath" not in device.keys():
                self.stop_all(f"Error: Binary firmware location not specified for {device_name}")

            # If we want to use the native desktop binary for a device, instead of
            # a connected Teensy, we can do that by wrapping a serial port around it.
            if device['run_mode'] == 'native':
                master_fd, slave_fd = pty.openpty()
                binary_process = subprocess.Popen(device['binary_filepath'], stdout=master_fd, stderr=master_fd, stdin=master_fd)
                self.binaries.append({
                    "subprocess": binary_process,
                    "pty_master_fd": master_fd,
                    "pty_slave_fd": slave_fd,
                })
                device['port'] = os.ttyname(slave_fd)
                device['baud_rate'] = 9600

            # Generate data loggers and device manager for device
            device_datastore = Datastore(device_name, self.simulation_run_dir)
            device_logger = Logger(device_name, self.simulation_run_dir)
            port_cmd = StateSession(device_name, device_datastore, device_logger)

            # Connect to device, failing gracefully if device connection fails
            if port_cmd.connect(device["port"], device["baud_rate"]):
                self.devices[device_name] = port_cmd
                self.datastores[device_name] = device_datastore
                self.loggers[device_name] = device_logger

                device_datastore.start()
                device_logger.start()
            elif device["required"]:
                self.stop_all("Error: a required serial port is disconnected.")

        # Set up MATLAB simulation
        self.sim = Simulation(self.devices, self.random_seed)
        self.sim.start(self.sim_duration)

        # Set up user command prompt
        cmd_prompt = StateCmdPrompt(self.devices, self.stop_all)
        cmd_prompt.intro = "Beginning console.\nType \"help\" for a list of commands."
        cmd_prompt.prompt = '> '
        try:
            cmd_prompt.cmdloop()
        except KeyboardInterrupt:
            # Gracefully exit session
            cmd_prompt.do_quit(None)
            self.stop_all("Exiting due to keyboard interrupt.")

    def stop_all(self, reason_for_stop):
        """Gracefully ends simulation run."""

        print(reason_for_stop)

        print("Stopping simulation...")
        self.sim.stop(self.simulation_run_dir)

        print("Stopping loggers (please be patient)...")
        for datastore in self.datastores.values():
            datastore.stop()
        for logger in self.loggers.values():
            logger.stop()

        print("Terminating device connections...")
        for binary in self.binaries:
            binary['subprocess'].terminate()
            os.close(binary['pty_master_fd'])
            os.close(binary['pty_slave_fd'])
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

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    parser.add_argument('-d', '--data-dir', action='store',
        help='''Directory for storing run data. Must be an absolute path. Default is logs/ relative to this script's location on disk.
                For the current run, a subdirectory of DATA_DIR is created in which the actual data is stored.''', default=log_dir)
    args = parser.parse_args()

    try:
        with open(args.conf, 'r') as config_file:
            config_data = json.load(config_file)
            device_config = config_data["devices"]
            random_seed = config_data["seed"]
            sim_duration = config_data["sim_duration"]
    except json.JSONDecodeError:
        print("Could not load config file. Exiting.")
        raise SystemExit
    except KeyError:
        print("Malformed config file. Exiting.")
        raise SystemExit

    simulation_run = SimulationRun(random_seed, sim_duration, args.data_dir, device_config)
    simulation_run.start()
