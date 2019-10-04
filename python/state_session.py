import time
import datetime
import serial
import threading
import json
import traceback

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
        console_port: Serial port of connected Teensy
        data_dir: Directory in which to store the results of the run.
        device_name: Name of device being connected to
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

        self.console_port = console_port
        self.baud_rate = baud_rate

        try:
            self.console = serial.Serial(self.console_port, self.baud_rate)
            self.start_time = datetime.datetime.now() # This is t = 0 on the Teensy, +/- a few milliseconds.

            self.device_write_lock = threading.Lock() # Lock to prevent multiple writes to device at the same time.

            self.updating_data_lock = threading.Lock()     # Lock to allow for shared reads/writes of timeseries data between
            # the check_msgs thread and the save_data thread

            self.awaiting_value_cv = threading.Condition() # Used to notify read_state that its requested value has arrived
            self.awaiting_value = False
            self.awaited_value = None

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

    def _store_awaited_value(self, data):
        '''
        Helper method used by check_console_msgs to return requested values to the main
        state session thread.
        '''

        self.awaiting_value_cv.acquire()
        awaited_value_name = self.awaited_value_name
        self.awaiting_value_cv.release()
        if data['field'] == awaited_value_name:
            self.awaiting_value_cv.acquire()
            self.awaited_value = data['val']
            self.awaiting_value = False
            self.awaiting_value_cv.notify()
            self.awaiting_value_cv.release()

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
                    self._store_awaited_value(data)
                else:
                    # If the 'read state' command is awaiting the current field's value,
                    # report it!
                    self._store_awaited_value(data)
                    self.datastore.put(data)

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

    def read_state(self, field_name, timeout = None):
        '''
        Read state.
        
        Read the value of the state field associated with the given field name on the flight controller.
        '''

        json_cmd = {'mode': ord('r'), 'field': str(field_name)}
        self.device_write_lock.acquire()
        self.console.write(json.dumps(json_cmd).encode())
        self.device_write_lock.release()

        # Wait for value to be found by check_console_msgs
        with self.awaiting_value_cv as cv:
            self.awaiting_value_cv.acquire()
            self.awaited_value_name = field_name
            self.awaiting_value_cv.release()
            while self.awaiting_value:
                cv.wait(timeout = timeout)

        return self.awaited_value

    def _write_state_basic(self, field_name, val):
        '''
        Write state.

        Overwrite the value of the state field with the given state field name on the flight controller.
        '''

        json_cmd = {
            'mode': ord('w'),
            'field': str(field_name),
            'val': str(val)
        }
        self.device_write_lock.acquire()
        self.console.write(json.dumps(json_cmd).encode())
        self.device_write_lock.release()

    def write_state(self, field_name, val):
        '''
        Write state.

        Overwrite the value of the state field with the given state field name on the flight computer.
        If the value is being overriden by user input, no changes are applied to computer state.

        This function doesn't check for the value of the state actually getting set. That can be handled by
        wsfb().
        '''

        if field_name in self.overriden_variables:
            return
        self._write_state_basic(field_name, val)

    def write_state_fb(self, field_name, val, timeout = None):
        '''
        Write state and check write operation with feedback.

        Overwrite the value of the state field with the given state field name on the flight computer, and
        then verify (via a read request) that the state was actually set.
        '''

        self.write_state(field_name, val)
        return val == self.read_state(field_name, timeout)

    def override_state(self, field_name, val):
        '''
        Override simulation state.

        This function works the same way as ws(), but any fields modified through this function are added
        to a list of overriden variables. Once variables are in this list, ws() cannot be used to modify their
        state. This prevents Simulation, which only uses ws(), from writing state to the device. The reason
        for this function's existence is to allow manual overrides of state variables during operation, e.g.
        via a command prompt or within a test case definition.
        '''

        self.overriden_variables.add(field_name)
        self._write_state_basic(field_name, val)

    def release_override(self, field_name):
        ''' Release override of simulation state. '''

        try:
            self.overriden_variables.remove(field_name)
        except KeyError:
            # It doesn't matter if you try to take the override off of a field that wasn't
            # actually being written, so just ignore it.
            return

    def override_state_fb(self, field_name, val, timeout = None):
        '''
        Override state and check write operation with feedback.

        Behaves the same way as wsfb(), but is strictly written for a state variable that is overriden.
        '''

        self.override_state(field_name, val)
        return val == self.read_state(field_name, timeout)

    def disconnect(self):
        '''Quits the program and stores message log and field telemetry to file.'''

        print(f'Terminating console connection to and saving logging/telemetry data for {self.device_name}.')

        # End threads if there was actually a connection to the device
        if self.connected:
            self.running_logger = False
            self.check_msgs_thread.join()
            self.console.close()
