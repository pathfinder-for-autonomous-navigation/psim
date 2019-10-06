import time
import datetime
import serial
import threading
import json
import traceback
import queue

class StateSession(object):
    '''
    Represents a connection session with a Flight Computer's state system.

    This class is used by the simulation software and user command prompt to read and write to a
    flight computer's state.

    This object is thread-safe; if an instance of this class is shared between the MATLAB simulation
    interface (an instance of Simulation) and the user command line (an instance of StateCmdPrompt),
    they won't trip over each other in setting/receiving variables from the connected flight computer.
    '''

    def __init__(self, data_dir, device_name, datastore, logger):
        '''
        Initializes state cmd prompt.

        Args:
        data_dir: Directory in which to store the results of the run.
        device_name: Name of device being connected to
        datastore: Datastore to which telemetry data will be published
        logger: Logger to which log lines should be committed
        '''

        # Device connection
        self.device_name = device_name
        self.connected = False

        # Data logging
        self.datastore = datastore
        self.logger = logger

        # Simulation
        self.overriden_variables = set()

    def connect(self, console_port, baud_rate=1152000, msg_check_delay=0.005):
        '''
        Starts serial connection to the flight computer.

        Args:
        - console_port: Serial port to connect to.
        - baud_rate: Baud rate of connection (default 1152000)
        - msg_check_delay: Delay between looped checks of messages from the flight controller. (default 5.0 ms)
        '''
        try:
            self.console = serial.Serial(console_port, baud_rate)
            self.start_time = datetime.datetime.now() # This is t = 0 on the Teensy, +/- a few milliseconds.

            self.device_write_lock = threading.Lock() # Lock to prevent multiple writes to device at the same time.

            # Queues used to manage interface between the check_msgs_thread and calls to read_state or write_state
            self.field_request = queue.Queue(1)
            self.field_response = queue.Queue(1)

            self.running_logger = True
            self.check_msgs_thread = threading.Thread(
                name=f"{self.device_name} logger thread",
                target=self.check_console_msgs,
                args=[msg_check_delay])
            self.check_msgs_thread.start()

            self.connected = True
            print(f"Opened connection to {self.device_name}")
            return True
        except serial.SerialException:
            print(f"Unable to open serial port for {self.device_name}.")

            self.connected = False
            return False

    def check_console_msgs(self, delay):
        '''
        Read FC output for debug messages and state variable updates. Record debug messages
        to the logging file, and update the console's record of the state.

        Args:
        - delay: Time to wait between executions of the message check loop.
        '''

        while self.running_logger:
            logline = ''
            try:
                # Read line coming from device and parse it
                if self.console.inWaiting() > 0:
                    line = self.console.readline().rstrip()
                    data = json.loads(line)
                else:
                    time.sleep(delay)
                    continue

                data['time'] = self.start_time + datetime.timedelta(milliseconds=data['t'])

                if 'msg' in data:
                    # The logline represents a debugging message created by Flight Software. Report the message to the logger.
                    logline = f"[{data['time']}] ({data['svrty']}) {data['msg']}"
                elif 'err' in data:
                    # The log line represents an error in retrieving or writing state data that
                    # was caused by a StateSession client improperly setting/retrieving a value.
                    # Report this failure to the logger.

                    logline = f"[{data['time']}] (ERROR) Tried to {data['mode']} state value named \"{data['field']}\" but encountered an error: {data['err']}"

                    data['val'] = None
                else:
                    # A valid telemetry field was returned. Manage it.
                    self.datastore.put(data)

                if self.field_request.queue[0] == data['field']:
                    self.field_request.get()  # Clear the field request
                    self.field_response.put(data)

            except ValueError:
                logline = f'[RAW] {line}'
            except serial.SerialException:
                print('Error: unable to read serial port for {}. Exiting.'.
                      format(self.device_name))
                self.disconnect()
            except:
                traceback.print_exc()
                print('Unspecified error. Exiting.')
                self.disconnect()

            self.logger.put(logline)

            time.sleep(delay)

    def _wait_for_state(self, field_name, timeout = None):
        """
        Helper function used by both read_state and write_state to wait for a desired value
        to be reported back by the Teensy (or native binary.)
        """
        self.field_request.put(field_name)
        try:
            data = self.field_response.get(True, timeout)
            return data['val']
        except queue.Empty:
            return None

    def read_state(self, field_name, timeout = None):
        '''
        Read state.
        
        Read the value of the state field associated with the given field name on the flight controller.
        '''

        json_cmd = {'mode': ord('r'), 'field': str(field_name)}
        json_cmd = json.dumps(json_cmd) + "\n"
        self.device_write_lock.acquire()
        self.console.write(json_cmd.encode())
        self.device_write_lock.release()

        return self._wait_for_state(field_name)

    def _write_state_basic(self, field_name, val, timeout = None):
        '''
        Write state.

        Overwrite the value of the state field with the given state field name on the flight controller.
        '''

        json_cmd = {
            'mode': ord('w'),
            'field': str(field_name),
            'val': str(val)
        }
        json_cmd = json.dumps(json_cmd) + "\n"
        self.device_write_lock.acquire()
        self.console.write(json_cmd.encode())
        self.device_write_lock.release()

        return val == self._wait_for_state(field_name, timeout)

    def write_state(self, field_name, val, timeout = None):
        '''
        Write state and check write operation with feedback.

        Overwrite the value of the state field with the given state field name on the flight computer, and
        then verify that the state was actually set. Do not write the state if the variable is being overriden
        by the user. (This is the function that sim should exclusively use.)
        '''

        if field_name in self.overriden_variables:
            return
        return self._write_state_basic(field_name, val, timeout)

    def override_state(self, field_name, val, timeout = None):
        '''
        Override state and check write operation with feedback.

        Behaves the same way as write_state_fb(), but is strictly written for a state variable that is overriden
        by the user, i.e. is no longer set by the simulation.
        '''

        self.overriden_variables.add(field_name)
        return self._write_state_basic(field_name, val, timeout)

    def release_override(self, field_name):
        ''' Release override of simulation state. '''

        try:
            self.overriden_variables.remove(field_name)
        except KeyError:
            # It doesn't matter if you try to take the override off of a field that wasn't
            # actually being overridden, so just ignore it.
            return

    def disconnect(self):
        '''Quits the program and stores message log and field telemetry to file.'''

        print(f'Terminating console connection to and saving logging/telemetry data for {self.device_name}.')

        # End threads if there was actually a connection to the device
        if self.connected:
            self.running_logger = False
            self.check_msgs_thread.join()
            self.console.close()
