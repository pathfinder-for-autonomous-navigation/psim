from usb_console.data_containers import Datastore, Logger
import os, json, tempfile, time

def test_datastore():
    tempdir = tempfile.mkdtemp() + "/datastore_test_dir"
    os.makedirs(tempdir, exist_ok=True)
    datastore = Datastore("test_device", tempdir)
    datastore.start()
    datastore.put({"field": "test_field", "val" : "val", "time" : "2"})
    time.sleep(1.5) # Allow for some time for the data to be processed by the queue processor
    datastore.stop()

    with open(tempdir + "/test_device-telemetry.txt","r") as datafile:
        data = json.load(datafile)
        assert 'test_field' in data.keys()
        assert len(data['test_field']) == 1

        datapoint = data['test_field'][0]
        assert datapoint[0] == "2"
        assert datapoint[1] == "val"

def test_logger():
    tempdir = tempfile.mkdtemp() + "/logger_test_dir"
    os.makedirs(tempdir, exist_ok=True)
    logger = Logger("test_device", tempdir)
    logger.start()
    logger.put("Hello world")
    time.sleep(1.5) # Allow for some time for the data to be processed by the queue processor
    logger.stop()

    with open(tempdir + "/test_device-log.txt", "r") as logfile:
        log = logfile.read()
        assert log == "Hello world\n"
