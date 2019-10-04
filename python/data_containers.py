import queue

class DataContainer(object):
    def __init__(self, device_name, data_dir):
        self.device_name = device_name
        self.data_dir = data_dir
        self.queue = queue.Queue()

    def start(self):
        pass

    def save(self):
        """
        Clean up file used for the data container.
        """
        raise NotImplementedError

    def stop(self):
        self.save()

    def put(self, item):
        self.queue.put(item)

    def process_queue(self):
        """Take items put into the queue by a StateSession, clean them up, and pop them from the queue."""
        raise NotImplementedError

    def intermediate_save(self):
        """
        Save data container to a file intermittently (so that data isn't lost)
        """
        raise NotImplementedError

class Datastore(DataContainer):
    pass
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.data = {}

    def process_queue(self):
        while True:
            item = self.queue.get()
            datapoint = (item['time'], item['val'])
            self.data[item['field']].append(datapoint)
            self.queue.task_done()

    def intermediate_save(self):
        pass

    def save(self):
        """
        Clean up file used for the data logger
        """
        pass

class Logger(DataContainer):
    def __init__(self, device_name, data_dir):
        super().__init__(device_name, data_dir)
        self.log = ""

    def process_queue(self):
        while True:
            logline = self.queue.get()
            self.log += logline + "\n"
            self.queue.task_done()

    def intermediate_save(self):
        pass

    def save(self):
        pass
