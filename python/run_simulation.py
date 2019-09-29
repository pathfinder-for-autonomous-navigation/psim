#!/usr/local/bin/python3

from argparse import ArgumentParser
from state_session import StateSession
from cmdprompt import StateCmdPrompt
from data_containers import Logger, Simulation
from simulation import Simulation

class SimulationRun(object):
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.datastores = {}
        self.loggers = {}
        self.devices = {}

    def start(self, devices):
        if not ports:
            print('Error: must specify at least one serial port.')
            raise SystemExit

        # Set up test table
        for device in devices:
            device_name = device["name"]
            device_datastore = Datastore(device_name, self.data_dir)
            device_logger = Logger(device_name, self.data_dir)
            port_cmd = StateSession(data_dir, device_name, device_datastore, device_logger)
            if port_cmd.connect(device["port"], 1152000):
                self.devices[device_name] = port_cmd
                self.datastores[device_name] = device_datastore
                self.loggers[device_name] = device_logger

                device_datastore.start()
                device_logger.start()
            else:
                print("Could not connect to {}. Exiting.".format(device_name))
                self.stop_data_containers()
                return

        # Set up MATLAB simulation
        sim = Simulation(self.devices)
        sim_thread = threading.Thread(name="Python-MATLAB Simulation Interface", target=sim.run)
        sim_thread.start()

        # User command prompt
        cmd_prompt = StateCmdPrompt(self.devices)
        cmd_prompt.intro = open('pan_logo.txt', 'r').read()
        cmd_prompt.prompt = '> '
        try:
            cmd_prompt.cmdloop()
        except KeyboardInterrupt:
            # Gracefully exit session
            cmd_prompt.do_quit(None)
            self.stop_data_containers()

    def stop_data_containers(self):
        for data_container in self.datastores + self.loggers:
            data_container.stop()
        for device in self.devices.values():
            device.disconnect()


if __name__ == '__main__':
    parser = ArgumentParser(description='''
    Interactive console allows sending state commands to PAN Teensy devices, and parses console output 
    from Teensies into human-readable, storable logging information.''')

    parser.add_argument('-c', '--conf', action='store', help='JSON file listing serial ports and Teensy computer names.',
                        required=True)
    parser.add_argument('-d', '--data-dir', action='store',
        help='''Directory for storing run data, relative to the location of the console script. Default is logs/.
                A subdirectory is created in which the actual data is stored.''', default='logs')
    args = parser.parse_args()

    try:
        devices = json.load(args.conf)
    except:
        print("Could not load JSON file. Exiting.")
        raise SystemExit

    simulation_run = SimulationRun(data_dir)
    simulation_run.start(devices)
