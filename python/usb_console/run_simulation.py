#!/usr/local/bin/python3

from argparse import ArgumentParser
from state_session import StateSession
from radio_session import RadioSession
from cmdprompt import StateCmdPrompt
from data_consumers import Logger, Datastore
from simulation import Simulation
import json, sys, os, tempfile, time

try:
    import pty, subprocess
except ImportError:
    # The current OS is Windows, and pty doesn't exist
    pass

class SimulationRun(object):
    def __init__(self, random_seed, sim_duration, data_dir, device_config, radios_config, radio_keys_config, flask_keys_config):
        self.random_seed = random_seed
        self.sim_duration = sim_duration

        self.simulation_run_dir = os.path.join(data_dir, time.strftime("%Y%m%d-%H%M%S"))
        # Create directory for run data
        os.makedirs(self.simulation_run_dir, exist_ok=True)

        self.device_config = device_config
        self.radios_config = radios_config
        self.radio_keys_config = radio_keys_config
        self.flask_keys_config = flask_keys_config

        self.datastores = {}
        self.loggers = {}
        self.devices = {}
        self.radios = {}
        self.binaries = []

    def start(self):
        """Starts a run of the simulation."""

        pan_logo_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pan_logo.txt')
        with open(pan_logo_filepath, 'r') as pan_logo_file:
            print(pan_logo_file.read())

        if not self.device_config:
            print('must specify at least one serial port.')
            raise SystemExit

        self.set_up_devices()
        self.set_up_radios()
        self.set_up_sim()
        self.set_up_cmd_prompt()

    def set_up_devices(self):
        # Set up test table by connecting to each device specified in the config.
        for device in self.device_config:
            try:
                device_name = device["name"]
            except:
                self.stop_all("Invalid configuration file. A device's name was not specified.")

            # Check device configuration.
            if device.get('run_mode') not in ['teensy', 'native']:
                self.stop_all(f"Device configuration for {device_name} is invalid.")
            if device['run_mode'] == "teensy":
                if not device.get("baud_rate"):
                    self.stop_all(f"device configuration for {device_name} does not specify baud rate.")
            elif "binary_filepath" not in device.keys():
                self.stop_all(f"Binary firmware location not specified for {device_name}")

            # If we want to use the native desktop binary for a device, instead of
            # a connected Teensy, we can do that by wrapping a serial port around it.
            if device['run_mode'] == 'native':
                try:
                    master_fd, slave_fd = pty.openpty()
                    binary_process = subprocess.Popen(device['binary_filepath'], stdout=master_fd, stderr=master_fd, stdin=master_fd)
                    self.binaries.append({
                        "subprocess": binary_process,
                        "pty_master_fd": master_fd,
                        "pty_slave_fd": slave_fd,
                    })
                    device['port'] = os.ttyname(slave_fd)
                    device['baud_rate'] = 9600
                except NameError:
                    # pty isn't defined because we're on Windows
                    self.stop_all(f"Cannot connect to a native binary for device {device_name}, since the current OS is Windows.")

            # Generate data loggers and device manager for device
            device_datastore = Datastore(device_name, self.simulation_run_dir)
            device_logger = Logger(device_name, self.simulation_run_dir)
            device_session = StateSession(device_name, device_datastore, device_logger)

            # Connect to device, failing gracefully if device connection fails
            if device_session.connect(device["port"], device["baud_rate"]):
                self.devices[device_name] = device_session
                self.datastores[device_name] = device_datastore
                self.loggers[device_name] = device_logger

                device_datastore.start()
                device_logger.start()
            else:
                self.stop_all("A required device is disconnected.")

    def set_up_radios(self):
        for radio in self.radios_config:
            try:
                radio_connected_device = radio["connected_device"]
                radio_name = radio["connected_device"] + "Radio"
            except:
                self.stop_all("Invalid configuration file. A radio's connected device was not specified.")

            # Check radio configuration. adjust path to radio_keys.json config file
            if 'imei' not in radio_keys_config:
                self.stop_all(f"IMEI number for radio connected to {radio_connected_device} was not specified.")
            if 'connect' not in radio.keys():
                self.stop_all(f"Configuration for {radio_connected_device} does not specify whether or not to connect to the radio.")

            if radio['connect']:
                radio_data_name = radio_connected_device + "_radio"
                radio_datastore = Datastore(radio_data_name, self.simulation_run_dir)
                radio_logger = Logger(radio_data_name, self.simulation_run_dir)
                radio_session = RadioSession(radio_connected_device, radio_datastore, radio_logger, self.radio_keys_config, self.flask_keys_config)

                #if radio_session.connect(radio['imei']):
                if radio_session.connect():
                    self.radios[radio_name] = radio_session
                    self.datastores[radio_data_name] = radio_datastore
                    self.loggers[radio_data_name] = radio_logger

                    radio_datastore.start()
                    radio_logger.start()
                else:
                    self.stop_all(f"Unable to connect to radio for {radio_connected_device}.")

    def set_up_sim(self):
        if self.sim_duration > 0:
            self.sim = Simulation(self.devices, self.random_seed)
            self.sim.start(self.sim_duration)
        else:
            self.sim = lambda: None # Create empty object
            self.sim.running = False

    def set_up_cmd_prompt(self):
        # Set up user command prompt
        cmd_prompt = StateCmdPrompt(self.devices, self.radios, self.sim, self.stop_all)
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

        print("Error: " + reason_for_stop)

        print("Stopping simulation (please be patient)...")
        try:
            self.sim.stop(self.simulation_run_dir)
        except:
            # Simulation was never created
            pass

        print("Stopping loggers (please be patient)...")
        for datastore in self.datastores.values():
            datastore.stop()
        for logger in self.loggers.values():
            logger.stop()

        num_radios = len(self.radios.values())
        print(f"Terminating {num_radios} radio connection(s)...")
        for radio in self.radios.values():
            radio.disconnect()

        num_devices = len(self.devices.values())
        print(f"Terminating {num_devices} device connection(s)...")
        for device in self.devices.values():
            device.disconnect()
        for binary in self.binaries:
            binary['subprocess'].terminate()
            os.close(binary['pty_master_fd'])
            os.close(binary['pty_slave_fd'])

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
            random_seed = config_data["seed"]
            sim_duration = config_data["sim_duration"]
            device_config = config_data["devices"]
            radios_config = config_data["radios"]
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'configs/radio_keys.json')) as radio_keys_config_file:
            radio_keys_config = json.load(radio_keys_config_file)
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'configs/flask_keys.json')) as flask_keys_config_file:
            flask_keys_config = json.load(flask_keys_config_file)
    except json.JSONDecodeError:
        print("Could not load config file. Exiting.")
        raise SystemExit
    except KeyError:
        print("Malformed config file. Exiting.")
        raise SystemExit

    simulation_run = SimulationRun(random_seed, sim_duration, args.data_dir,
                                   device_config, radios_config, radio_keys_config, flask_keys_config)
    simulation_run.start()
