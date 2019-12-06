from flask import Flask, request, jsonify
from flasgger import Swagger
from flasgger.utils import swag_from
from elasticsearch import Elasticsearch
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
        #need to move this function to radioSession
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
        once downlink producer is finished
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

                    #handles downlinks
                    if True:
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
                                            self.send_uplinks=True

                            #check if there is an email attachment
                            if part.get_filename() is not None:
                                #get data from email attachment
                                attachmentContents=part.get_payload(decode=True).decode('utf8')
                                statefield_report=self.process_downlink_packet(attachmentContents)
                                return statefield_report

    def post_to_es(self):
        '''
        Check for the most recent email from iridium.
        If there is a statefield report, index the statefield report and 
        create and index an iridium report.
        '''
        #connect to elasticsearch
        es=Elasticsearch([{'host':self.es_server,'port':self.es_port}])

        while self.run_email_thread==True:
            #print the report recieved by email 
            sf_report=readIr.check_for_email()

            if sf_report is not None:
                print("Got report: "+str(sf_report)+"\n\n")

                #index statefield report in elasticsearch
                statefield_res = es.index(index='statefield_report', doc_type='report', body=sf_report)

                # Create an iridium report and add that to the iridium_report index in elasticsearch. 
                # We only want to add Iridium reports when we recieve an email, which is why we wait until there is a statefield report
                ir_report=json.dumps({"momsn":self.momsn, "mtmsn":self.mtmsn})
                iridium_res = es.index(index='iridium_report', doc_type='report', body=ir_report)

                #print whether or not indexing was successful
                print(statefield_res['result']+"\n"+iridium_res['result'])

    def disconnect(self):
        '''
        Stops the thread which is checking emails
        and posting reports to elasticsearch
        '''
        self.run_email_thread=False
        self.check_email_thread.join()

#get keys for connecting to email account and elasticsearch server
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

#create a read_iridium object
readIr=read_iridium(radio_keys_config, server_keys_config)
#start checking emails and posting reports
readIr.connect()

#set up the SwaggerUI API
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

#Endpoint for testing post requests
@app.route("/test", methods=["POST"])
@swag_from("endpoint_configs/echo_config.yml")
def echo():
    input_json=request.get_json()
    message=str(input_json["message"])
    res={"Recieved": message}
    return json.dumps(res)

#endpoint for indexing telemetry data in elasticsearch
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

if __name__ == "__main__":
    app.run(debug=True)




#gets token for session. honestly doesn't seem like we even need this, but we shall see?
'''
headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
}
login={
    "username":username,
    "password":password
}
login_json=json.dumps(login)
response = requests.post('http://'+readIr.server+":"+readIr.port+"/auth/login", headers=headers, data=login_json)
response_body=json.loads(response.text)
token=response_body["token"]
'''