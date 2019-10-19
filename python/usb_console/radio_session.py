import time
import datetime
import serial
import threading
import json
import traceback
import queue


class RadioSession(object):
    '''
    Represents a connection session with a Flight Computer's Quake radio.

    This class is used by the simulation software and user command prompt to read and write to a
    flight computer's Quake radio. The connection is fundamentally stateless, since the state of the
    connection (connected or disconnected) is managed by Iridium.

    TODO this class needs to be thread-safe. Protect uplinks via a lock, and protect data sharing
    between the check_for_downlink and the read_state functions.
    '''

    def __init__(self, device_name, datastore, logger):
        '''
        Initializes state session with the Quake radio.

        Args:
        device_name: Name of device being connected to
        datastore: Datastore to which telemetry data will be published
        logger: Logger to which log lines should be committed
        '''

        # Device connection
        self.device_name = device_name

        # Data logging
        self.datastore = datastore
        self.logger = logger

        # Simulation
        self.overriden_variables = set()

    def connect(self, imei):
        '''
        Starts serial connection to the desired device.

        Args:
        - imei: IMEI number of Quake to connect to.
        '''
        self.imei = imei
        self.check_downlink_thread = threading.Thread(target=self.check_for_downlink)
        self.running_logger = True
        self.check_downlink_thread.start()

        print(f"Opened connection to {self.device_name} radio.")
        return True

    def check_for_downlink(self):
        '''
        Read device output for debug messages and state variable updates. Record debug messages
        to the logging file, and update the console's record of the state.
        '''

        while self.running_logger:
            # TODO implement
            pass

    def read_state(self, field, timeout=None):
        '''
        Read state.
        
        Read the latest value of the state field received via downlink with the name "field".
        '''
        raise NotImplementedError

    def write_multiple_states(self, fields, vals, timeout=None):
        '''
        Uplink multiple state variables. Return success of write.
        '''
        assert len(fields) == len(vals)

        # TODO send uplink via email.
        raise NotImplementedError

    def write_state(self, field, val, timeout=None):
        '''
        Uplink one state variable. Return success of write.
        '''
        return self.write_multiple_states([field], [val], timeout)

    def disconnect(self):
        '''Quits the Quake connection, and stores message log and field telemetry to file.'''

        print(
            f' - Terminating console connection to and saving logging/telemetry data for radio connection to {self.device_name}.'
        )

        # End threads if there was actually a connection to the radio
        self.running_logger = False
        self.check_downlink_thread.join()