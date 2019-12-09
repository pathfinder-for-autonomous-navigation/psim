import time
import datetime
import serial
import threading
import json
import traceback
import queue
import yagmail
import imaplib
import base64
import os
import email
import requests


class RadioSession(object):
    '''
    Represents a connection session with a Flight Computer's Quake radio.

    This class is used by the simulation software and user command prompt to read and write to a
    flight computer's Quake radio. The connection is fundamentally stateless, since the state of the
    connection (connected or disconnected) is managed by Iridium.

    TODO this class needs to be thread-safe. Protect uplinks via a lock, and protect data sharing
    between the check_for_downlink and the read_state functions.
    '''

    def __init__(self, device_name, datastore, logger, radio_keys_config, flask_keys_config):
        '''
        Initializes state session with the Quake radio.

        Args:
        device_name: Name of device being connected to
        datastore: Datastore to which telemetry data will be published
        logger: Logger to which log lines should be committed
        '''

        # Device connection
        self.device_name = device_name

        #Flask server connection
        self.flask_server=flask_keys_config["server"]
        self.flask_port=flask_keys_config["port"]

        # Data logging
        self.datastore = datastore
        self.logger = logger

        #email
        self.username=radio_keys_config["email_username"]
        self.password=radio_keys_config["email_password"]

        #imei
        self.imei=radio_keys_config["imei"]

        # Simulation
        self.overriden_variables = set()

    def read_state(self, field, timeout=None):
        '''
        Read state by posting a request for data to the Flask server
        '''
        headers = {
            'Accept': 'text/html',
        }

        response = requests.get('http://'+self.flask_server+':'+self.flask_port+'/search-statefields?imei='+str(self.imei)+'&field='+str(field), headers=headers)
        return response.text

    def write_multiple_states(self, fields, vals, timeout=None):
        '''
        Uplink multiple state variables. Return success of write.
        Reads from the most recent Iridium Report whether or
        not RadioSession is able to send uplinks
        '''
        assert len(fields) == len(vals)

        headers = {
            'Accept': 'text/html',
        }

        response = requests.get('http://'+self.flask_server+':'+self.flask_port+'/search-iridium?imei='+str(self.imei)+'&field=send-uplinks', headers=headers)
        print(response.text)

        if response.text=="True":
            #create dictionary object with new fields and vals
            updated_fields={}
            for i in range(len(fields)):
                updated_fields[fields[i]]=vals[i]

            #connect to PAN email account
            yag = yagmail.SMTP(self.username, self.password)

            #create a JSON file with the updated statefields and send it to the iridium email
            with open('uplink.json', 'w') as json_uplink:
                json.dump(updated_fields, json_uplink)
            yag.send('sbdservice@sbd.iridium.com', self.imei, 'uplink.json')

            return True
        else:
            self.logger.put("Wait for confirmation MTMSN")
            return False

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
