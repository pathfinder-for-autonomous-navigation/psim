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

    def __init__(self, device_name, datastore, logger, radio_keys_config):
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

        #email
        self.username=radio_keys_config["email_username"]
        self.password=radio_keys_config["email_password"]

        #imei
        self.imei=radio_keys_config["imei"]

        # Simulation
        self.overriden_variables = set()

        #updates MOMSN aand MTMSN numbers sent/recieved
        self.momsn=-1
        self.mtmsn=-1
        self.confirmation_mtmsn=-1
        #send_uplinks keeps track of if the ground can send more uplinks. This allows us to make sure we are only sending one uplink at a time.
        self.send_uplinks=True

        self.statefields={
            #dummy variable for statefields
        }

    def connect(self, imei):
        '''
        Starts serial connection to the desired device.

        Args:
        - imei: IMEI number of Quake to connect to.
        '''
        self.imei = imei
        self.check_email_thread = threading.Thread(target=self.check_for_email)
        self.running_logger = True
        self.check_email_thread.start()

        print(f"Opened connection to {self.device_name} radio.")
        return True



    def check_for_email(self):
        '''
        Read device output for debug messages and state variable updates. Record debug messages
        to the logging file, and update the console's record of the state.
        '''
        
        #connect to PAN email account  
        try:
            mail = imaplib.IMAP4_SSL("imap.gmail.com", 993)
            mail.login(self.username, self.password)
            mail.select('Inbox')
            self.logger.put("Connected to Email")
        except:
            self.logger.put("Unable to connect to email")

        while self.running_logger:
            #.search() searches from mail. Data gives id's of all emails.
            _, data = mail.search(None, '(FROM "sbdservice@sbd.iridium.com")', '(UNSEEN)')
            mail_ids = data[0]
            id_list = mail_ids.split()
            self.processEmails(id_list, mail)

    #This is just deserializing JSON for now, but in the future will be replaced with the actual deserialization scheme for downlink packets
    def process_downlink_packet(self, data):
        return json.loads(data)

    def processEmails(self, id_list, mail):
        for num in id_list:
            #.fetch() fetches the mail for given id where 'RFC822' is an Internet 
            # Message Access Protocol.
            _, data = mail.fetch(num,'(RFC822)')
                
            #go through each component of data
            for response_part in data:
                if isinstance(response_part, tuple):
                    # converts message from byte literal to string removing b''
                    msg = email.message_from_string(response_part[1].decode('utf-8'))
                    email_subject = msg['subject']

                    #handles uplinks
                    if email_subject.find("SBD Mobile Terminated Message Queued for Unit: "+str(self.imei))==0:
                        for part in msg.walk():
                                
                            if part.get_content_maintype() == 'multipart':
                                continue

                            if part.get('Content-Disposition') is None:
                                continue
                            
                            #get the body text of the email and look for the MOMSN/MTMSN number
                            if part.get_content_type() == "text/plain":
                                email_body = part.get_payload(decode=True).decode('utf8')
                                for line in email_body.splitlines():
                                    if line.find("MTMSN")!=-1:
                                        self.mtmsn=int(line[line.find("MTMSN")+9:line.find("MTMSN")+11])

                    #handles downlinks
                    if email_subject.find("SBD Msg From Unit: "+str(self.imei))==0:
                        #go through the email contents
                        for part in msg.walk():
                                
                            if part.get_content_maintype() == 'multipart':
                                continue

                            if part.get('Content-Disposition') is None:
                                continue
                            
                            #get the body text of the email and look for the MOMSN/MTMSN number
                            if part.get_content_type() == "text/plain":
                                email_body = part.get_payload(decode=True).decode('utf8')
                                for line in email_body.splitlines():
                                    if line.find("MOMSN")!=-1:
                                        self.momsn=int(line[7:])
                                    if line.find("MTMSN")!=-1:
                                        self.confirmation_mtmsn=int(line[7:])
                                        if self.confirmation_mtmsn != self.mtmsn:
                                            #stop radio session from sending any more uplinks
                                            self.send_uplinks=False
                                        else:
                                            #allow radio session to send more uplinks
                                            self.stop_uplinks=True
                                            

                            #check if there is an email attachment
                            if part.get_filename() is not None:
                                #get data from email attachment
                                attachmentContents=part.get_payload(decode=True).decode('utf8')
                                data=self.process_downlink_packet(attachmentContents)
                                
                                #iterate through each statefield key of the dictionary
                                entry={}
                                for key in self.statefields:
                                    #if the value for a statefield exists in the json object recieved, update the dictionary and add entry to datastore
                                    if data.get(key)is not None:
                                        self.statefields[key]=data[key]
                                        entry['field'] = key
                                        entry['val'] = data[key]
                                        entry['time'] = datetime.datetime.now()
                                        self.datastore.consume_queue_item(entry)

    def read_state(self, field, timeout=None):
        '''
        Read state.
        
        Read the latest value of the state field received via downlink with the name "field".
        '''
        return self.statefields[field]

    def write_multiple_states(self, fields, vals, timeout=None):
        '''
        Uplink multiple state variables. Return success of write.
        '''
        assert len(fields) == len(vals)

        #create dictionary object with new fields and vals
        updated_fields={}
        for i in range(len(fields)):
            updated_fields[fields[i]]=vals[i]

        #connect to PAN email account
        yag = yagmail.SMTP(self.username, self.password)
        #create a JSON file with the updated statefields and send it to the iridium email
        with open('uplink.json', 'w') as json_uplink:
            json.dump(updated_fields, json_uplink)
        if self.send_uplinks==True:
            yag.send('sbdservice@sbd.iridium.com', 'Data', 'uplink.json')
        else:
            self.logger.put("Unable to send uplink")
        return True

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
        self.check_email_thread.join()
