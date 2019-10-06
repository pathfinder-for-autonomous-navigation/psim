from usb_console.data_containers import Datastore, Logger
import unittest
import os, json, tempfile

def test_datastore():
    tempdir = tempfile.mkdtemp() + "/datastore_test_dir"
    os.makedirs(tempdir, exist_ok=True)
    datastore = Datastore("test_device", tempdir)
    datastore.start()
    datastore.put({"field_name": "test_field", "val" : "val", "time" : "2"})
    datastore.stop()

    with open(tempdir + "/test_device-telemetry.txt","r") as datafile:
        data = json.load(datafile)
        assert 'test_field' in data.keys()
        assert len(data['test_field']) == 1

        datapoint = data['test_field'][0]
        assert datapoint['time'] == "2"
        assert datapoint['val'] == "val"

def test_logger():
    tempdir = tempfile.mkdtemp() + "/logger_test_dir"
    os.makedirs(tempdir, exist_ok=True)
    logger = Logger("test_device", tempdir)
    logger.start()
    logger.put("Hello world")
    logger.stop()

    with open(tempdir + "/test_device-log.txt", "r") as logfile:
        log = logfile.read()
        assert log == "Hello world\n"
