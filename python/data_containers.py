import Queue

class DataContainer(object):
    def __init__(self, device_name, data_dir):
        self.device_name = device_name
        self.data_dir = data_dir
        self.queue = Queue.Queue()

    def start(self):
        pass

    def stop(self):
        """
        Clean up file used for the data container.
        """
        raise NotImplementedError

    def put(self, item):
        self.queue.put(item)
    
    def process_queue(self):
        raise NotImplementedError

    def intermediate_save(self):
        """
        Save data container to a file intermittently (so that data isn't lost)
        """
        raise NotImplementedError

class Datastore(DataContainer):
    def __init__(self, data_dir):
        super.__init__(data_dir)
        self.data = {}

    def process_queue(self):
        for item in self.queue:
            datapoint = (item['time'], item['val'])
            self.data[item['field']].append(datapoint)

    def intermediate_save(self):
        pass

    def save(self):
        """
        Clean up file used for the data logger
        """
        pass

    def stop(self):
        self.save()

class Logger(DataContainer):
    def __init__(self, data_dir):
        super.__init__(data_dir)

    def intermediate_save(self):
        pass

    def stop(self):
        pass
