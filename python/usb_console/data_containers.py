import queue, os, json, threading, time

class DataContainer(object):
    def __init__(self, device_name, data_dir):
        self.device_name = device_name
        self.data_dir = data_dir
        self.queue = queue.Queue()

    def process_queue_item(self):
        """
        Takes an item put into the queue by a StateSession, and processes it.
        """
        raise NotImplementedError

    def process_queue(self):
        """ The data processing thread. """
        while self.running:
            try:
                while True:
                    item = self.queue.get_nowait()
                    self.process_queue_item(item)
            except queue.Empty:
                pass

            time.sleep(1.0) # Sleep 1 second

    def start(self):
        """ Start data processing thread. """
        self.processor_thread = threading.Thread(target=self.process_queue)
        self.running = True
        self.processor_thread.start()

    def save(self):
        """ Clean up file used for the data container. """
        raise NotImplementedError

    def stop(self):
        """ Stop data processing thread. """
        self.running = False
        self.processor_thread.join()
        self.save()

    def put(self, item):
        """
        External-facing method used by clients of this class to add data for processing.
        """
        self.queue.put(item)

class Datastore(DataContainer):
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.data = {}

    def process_queue_item(self, datapoint):
        """ Adds a single data point to the telemetry log."""

        time_value_pair = (str(datapoint['time']), datapoint['val'])
        field_name = datapoint['field']

        if not self.data.get(field_name):
            self.data[field_name] = []

        self.data[datapoint['field']].append(time_value_pair)

    def save(self):
        """ Save telemetry log to a file. """

        filename = f"{self.device_name}-telemetry.txt"
        filepath = os.path.join(self.data_dir, filename)

        with open(filepath, 'w') as datafile:
            json.dump(self.data, datafile)

class Logger(DataContainer):
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.log = ""

    def process_queue_item(self, logline):
        """Add a new line to the log."""

        self.log += logline + "\n"

    def intermediate_save(self):
        """Save the log generated thus far to a file."""

        filename = f"{self.device_name}-log.txt"
        filepath = os.path.join(self.data_dir, filename)
        with open(filepath, 'a') as logfile:
            logfile.write(self.log)
        self.log = ""

    def save(self):
        self.intermediate_save()
