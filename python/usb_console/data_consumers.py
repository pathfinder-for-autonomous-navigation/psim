import queue, os, json, threading, time
from datetime import datetime


class DataConsumer(object):
    def __init__(self, device_name, data_dir):
        self.device_name = device_name
        self.data_dir = data_dir
        self.queue = queue.Queue()

    def consume_queue_item(self, item):
        """
        Takes an item put into the queue by a StateSession, and consumes it.
        """
        raise NotImplementedError

    def consume_queue(self):
        """ The data consumer thread. """
        while self.running:
            try:
                while True:
                    item = self.queue.get_nowait()
                    self.consume_queue_item(item)
            except queue.Empty:
                pass

            time.sleep(1.0) # Sleep 1 second

    def start(self):
        """ Start data consumer thread. """
        self.consumer_thread = threading.Thread(target=self.consume_queue)
        self.running = True
        self.consumer_thread.start()

    def save(self):
        """ Save consumed data to disk. """
        raise NotImplementedError

    def stop(self):
        """ Stop data consumer thread. """
        self.running = False
        self.consumer_thread.join()
        self.save()

    def put(self, item):
        """
        External-facing method used by clients of this class to add data for consumption.

        Args:
         - item: Item to consume.
        """
        self.queue.put(item)


class Datastore(DataConsumer):
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.data = {}

    def consume_queue_item(self, datapoint):
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


class Logger(DataConsumer):
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.log = ""

    def consume_queue_item(self, logline):
        """Add a new line to the log."""
        self.log += logline + "\n"

    def save(self):
        """Save the log to a file."""

        filename = f"{self.device_name}-log.txt"
        filepath = os.path.join(self.data_dir, filename)
        with open(filepath, 'w') as logfile:
            logfile.write(self.log)
    
    def put(self, logline):
        logline="("+str(datetime.now())+")"+"\n"+logline+"\n"
        self.queue.put(logline)