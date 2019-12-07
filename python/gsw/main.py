from flask import Flask, request, jsonify
from flasgger import Swagger
from flasgger.utils import swag_from
from elasticsearch import Elasticsearch
from datetime import datetime
import threading
import json
import imaplib
import base64
import os
import email
import logging

app = Flask(__name__)
app.config["SWAGGER"]={"title": "PAN Ground Software", "uiversion": 2}
 
class read_iridium(object):
    def __init__(self, radio_keys_config, server_keys_config):
        #elasticsearch server
        self.es_server=server_keys_config["server"]
        self.es_port=server_keys_config["port"]

        #pan email
        self.username=radio_keys_config["email_username"]
        self.password=radio_keys_config["email_password"]

        #updates MOMSN and MTMSN numbers sent/recieved
        self.momsn=-1
        self.mtmsn=-1
        self.confirmation_mtmsn=-1

        #send_uplinks, keeps track of if the ground can send more uplinks. This allows us to make sure we are only sending one uplink at a time.
        self.send_uplinks=False

        #connect to email
        self.mail = imaplib.IMAP4_SSL("imap.gmail.com", 993)
        self.mail.login(self.username, self.password)
        self.mail.select('Inbox')

        #thread
        self.run_email_thread = False

    def connect(self):
        '''
        Starts a thread which will continuously
        check the PAN email and post reports to 
        elasticsearch 
        '''
        self.check_email_thread = threading.Thread(target=self.post_to_es)
        self.run_email_thread = True
        self.check_email_thread.start()

    def process_downlink_packet(self, data):
        '''
        Converts the email attachment data into a 
        JSON object. This is because elasticsearch
        only takes in JSON data. I will add to this
        once downlink producer is finished. 
        '''
        return json.loads(data)

    def check_for_email(self):
        '''
        Checks the PAN email account. Updates the most recent MOMSN 
        and MTMSN numbers. Returns the contents of the email attachement
        of the most recent unread email from the Iridium email account.
        '''
        #look for all new emails from iridium
        _, data = self.mail.search(None, '(FROM "fy56@cornell.edu")', '(UNSEEN)')
        mail_ids = data[0]
        id_list = mail_ids.split()

        for num in id_list:
            #.fetch() fetches the mail for given id where 'RFC822' is an Internet 
            # Message Access Protocol.
            _, data = self.mail.fetch(num,'(RFC822)')
            #mark the email message as read. I think this works?
            self.mail.store(num, '+FLAGS', '(\\Seen)')

            #go through each component of data
            for response_part in data:
                if isinstance(response_part, tuple):
                    # converts message from byte literal to string removing b''
                    msg = email.message_from_string(response_part[1].decode('utf-8'))
                    email_subject = msg['subject']

                    #handles uplink confirmations
                    if email_subject.find("SBD Mobile Terminated Message Queued for Unit: ")==0:
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

                    # Handles downlinks
                    if True:
                    #if email_subject.find("SBD Msg From Unit: ")==0:
                        # Go through the email contents
                        for part in msg.walk():
                                        
                            if part.get_content_maintype() == 'multipart':
                                continue

                            if part.get('Content-Disposition') is None:
                                continue
                                            
                            # Get the body text of the email and look for the MOMSN/MTMSN number
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
                                            self.send_uplinks=True

                            # Check if there is an email attachment
                            if part.get_filename() is not None:
                                # Get data from email attachment
                                attachmentContents=part.get_payload(decode=True).decode('utf8')
                                statefield_report=self.process_downlink_packet(attachmentContents)
                                return statefield_report

    def post_to_es(self):
        '''
        Check for the most recent email from iridium.
        If there is a statefield report, index the statefield report and 
        create and index an iridium report.
        '''
        # Connect to elasticsearch
        es=Elasticsearch([{'host':self.es_server,'port':self.es_port}])

        while self.run_email_thread==True:
            #get the most recent statefield report
            sf_report=readIr.check_for_email()

            if sf_report is not None:
                # Print the statefield report recieved
                print("Got report: "+str(sf_report)+"\n")

                # Index statefield report in elasticsearch
                statefield_res = es.index(index='statefield_report', doc_type='report', body=sf_report)
                # Print whether or not indexing was successful
                print("Statefield Report Status: "+statefield_res['result'])

                # Create an iridium report and add that to the iridium_report index in elasticsearch. 
                # We only want to add Iridium reports when we recieve an email, which is why we wait until there is a statefield report
                ir_report=json.dumps({
                    "momsn":self.momsn,
                    "mtmsn":self.mtmsn, 
                    "confirmation-mtmsn": self.confirmation_mtmsn,
                    "send-uplinks": self.send_uplinks,
                    "time": str(datetime.now().isoformat())
                })

                # Index iridium report in elasticsearch
                iridium_res = es.index(index='iridium_report', doc_type='report', body=ir_report)
                # Print whether or not indexing was successful
                print("Iridium Report Status: "+iridium_res['result']+"\n\n")

    def disconnect(self):
        '''
        Stops the thread which is checking emails
        and posting reports to elasticsearch
        '''
        self.run_email_thread=False
        self.check_email_thread.join()

# Get keys for connecting to email account and elasticsearch server
try:
    with open(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../usb_console/configs/radio_keys.json')))as radio_keys_config_file:
        radio_keys_config = json.load(radio_keys_config_file)
    with open(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../usb_console/configs/server_keys.json')))as server_keys_config_file:
        server_keys_config = json.load(server_keys_config_file)
except json.JSONDecodeError:
    print("Could not load config files. Exiting.")
    raise SystemExit
except KeyError:
    print("Malformed config file. Exiting.")
    raise SystemExit

# Create a read_iridium object
readIr=read_iridium(radio_keys_config, server_keys_config)
# Start checking emails and posting reports
readIr.connect()

# Set up the SwaggerUI API
swagger_config={
    "headers":[],
    "specs":[
        {
            "endpoint":"apispec_1",
            "route":"/apispec_1.json",
            "rule_filter":lambda rule:True,
            "model_filter":lambda tag:True
        }
    ],
    "static_url_path": "/flassger_static",
    "swagger_ui":True,
    "specs_route":"/swagger/"
}
swagger=Swagger(app, config=swagger_config)

# Endpoint for testing post requests
# Mostly for testing purposes. We don't use this to actually post data to elasticsearch
@app.route("/test", methods=["POST"])
@swag_from("endpoint_configs/echo_config.yml")
def echo():
    input_json=request.get_json()
    message=str(input_json["message"])
    res={"Recieved": message}
    return json.dumps(res)

# Endpoint for requesting data from elasicsearch. 
@app.route("/telemetry", methods=["POST"])
@swag_from("endpoint_configs/telemetry_config.yml")
def index_sf_report():
    #connect to elasticsearch
    es=Elasticsearch([{'host':readIr.es_server,'port':readIr.es_port}])

    sf_report=request.get_json()
    #index statefield report in elasticsearch
    sf_res = es.index(index='statefield_report', doc_type='report', body=sf_report)
    res={"Report Status": sf_res['result']}
    return res

# Endpoint for getting data from statefield reports
@app.route("/search-statefields", methods=["POST"])
@swag_from("endpoint_configs/search_statefields_config.yml")
def get_statefield():
    # Connect to elasticsearch
    es=Elasticsearch([{'host':readIr.es_server,'port':readIr.es_port}])

    input_json=request.get_json()
    field=str(input_json["statefield"])

    # Get the most recent document in the statefield index which has a given statefield in it
    search_object={
        'query': {
            'exists': {
                'field': field
            }
        },
        "sort": [
            {
                "at": {
                    "order": "desc"
                }
            }
        ],
        "size": 1
    }
    res = es.search(index='statefield_report', body=json.dumps(search_object))
    # Get the value of that statefield from the document
    most_recent_field=res["hits"]["hits"][0]["_source"][field]
    return most_recent_field

# Endpoint for getting data from iridium reports. Will be used to check if we are allowed to send uplinks
@app.route("/search-iridium", methods=["POST"])
@swag_from("endpoint_configs/search_iridium_config.yml")
def get_iridium_field():
    # Connect to elasticsearch
    es=Elasticsearch([{'host':readIr.es_server,'port':readIr.es_port}])

    input_json=request.get_json()
    field=str(input_json["field"])
    
    # Get the most recent document in the statefield index which has a given statefield in it
    search_object={
        'query': {
            'exists': {
                'field': field
            }
        },
        "sort": [
            {
                "time": {
                    "order": "desc"
                }
            }
        ],
        "size": 1
    }
    res = es.search(index='iridium_report', body=json.dumps(search_object))
    # Get the value of that statefield from the document
    most_recent_field=res["hits"]["hits"][0]["_source"][field]
    return str(most_recent_field)

if __name__ == "__main__":
    app.run(debug=True)