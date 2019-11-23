from flask import Flask, request, jsonify
from argparse import ArgumentParser
import threading
import time
import datetime
import json
import imaplib
import base64
import os
import email

class readIridium(object):
    def __init__(self, radio_keys_config):
        '''
        Initialize session with the Quake radio.
        Args:
        device_name: Name of device being connected to
        datastore: Datastore to which telemetry data will be published
        logger: Logger to which log lines should be committed
        '''

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
            "field": 20
        }
        
    def connect(self):
        '''
        Starts checking for emails
        '''
        self.check_email_thread = threading.Thread(target=self.check_for_email)
        self.check_email_thread.start()
        return True

    def check_for_email(self):
        '''
        Add description here later
        '''
            
        #connect to PAN email account  
        mail = imaplib.IMAP4_SSL("imap.gmail.com", 993)
        mail.login(self.username, self.password)
        mail.select('Inbox')

        #.search() searches from mail. Data gives id's of all emails. sbdservice@sbd.iridium.com
        _, data = mail.search(None, '(FROM "fy56@cornell.edu")', '(UNSEEN)')
        mail_ids = data[0]
        id_list = mail_ids.split()
        return self.processEmails(id_list, mail)

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

                    #handles uplink confirmations
                    if email_subject.find("SBD Mobile Terminated Message Queued for Unit: "==0:
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
                    if 1==1:
                    #if email_subject.find("SBD Msg From Unit: "==0:
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
                                return data
                                    
                                # not sure if we need this part anymore
                                # iterate through each statefield key of the dictionary.
                                entry={}
                                for key in self.statefields:
                                    #if the value for a statefield exists in the json object recieved, update the dictionary and add entry to datastore
                                    if data.get(key)is not None:
                                        self.statefields[key]=data[key]
                                        entry['field'] = key
                                        entry['val'] = data[key]
                                        entry['time'] = datetime.datetime.now()
                                        #return entry

app = Flask(__name__)

#get keys for connecting to email and Quake radio
try:
    with open(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../usb_console/configs/radio_keys.json')))as radio_keys_config_file:
        radio_keys_config = json.load(radio_keys_config_file)
except json.JSONDecodeError:
    print("Could not load radio keys. Exiting.")
    raise SystemExit
except KeyError:
    print("Malformed config file. Exiting.")
    raise SystemExit
    
#create a readIridium object and start checking for emails related to the Quake
readIr = readIridium(radio_keys_config)

#keep on posting data to clojure backend
@app.route("/", methods=['POST'])
def home():
    data=readIr.check_for_email()
    
    if data is not None:
        #url is api endpoint. i'm not sure what port it would be?
        url='206.189.193.31:'
        #need to figure out how to get login authorization since key/token expires
        key=""
        r = requests.post(url, data) 
        return r.text
    else:
        return ""

if __name__ == "__main__":
    app.run(debug=True)