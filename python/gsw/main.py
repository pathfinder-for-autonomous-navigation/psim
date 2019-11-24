from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO, send, emit
import requests
from threading import Thread,Event
import json
import imaplib
import base64
import os
import email
import sys

class readIridium(object):
    def __init__(self, radio_keys_config):

        #email
        self.username=radio_keys_config["email_username"]
        self.password=radio_keys_config["email_password"]

        #updates MOMSN aand MTMSN numbers sent/recieved
        self.momsn=-1
        self.mtmsn=-1
        self.confirmation_mtmsn=-1

        #send_uplinks keeps track of if the ground can send more uplinks. This allows us to make sure we are only sending one uplink at a time.
        self.send_uplinks=True
        
    def connect(self):
        '''
        Starts checking for emails
        '''
        mail = imaplib.IMAP4_SSL("imap.gmail.com", 993)
        mail.login(self.username, self.password)
        mail.select('Inbox')

        return mail

    #This is just deserializing JSON for now, but in the future will be replaced with the actual deserialization scheme for downlink packets
    def process_downlink_packet(self, data):
        return json.loads(data)

    def check_for_email(self, mail):
        _, data = mail.search(None, '(FROM "fy56@cornell.edu")', '(UNSEEN)')
        mail_ids = data[0]
        id_list = mail_ids.split()

        while not thread_stop_event.isSet():
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
                                    socketio.emit('new_msg', {'msg': data}, namespace='/test')
                                    return data

app = Flask(__name__)
socketio = SocketIO(app)

#get keys for connecting to email and Quake radio
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

#create a readIridium object and connect to mail account
readIr = readIridium(radio_keys_config)
pan_mail = readIr.connect()

#get keys for connecting to server
#server = server_keys_config["server"]
#auth = server_keys_config["auth"]
#username = auth["user"]
#password = auth["password"]

thread = Thread()
thread_stop_event = Event()

#use websocket to continuously push data to radio session
@app.route("/")
def home():
    #while readIr.start_thread == True:
    #email_data=readIr.check_for_email(pan_mail)
        
    #if email_data is not None:
        #r = requests.post(url, data) 
        #data=email_data
        #return data
    #else:
        #socketio.emit('emailmsg', {'data': data}, namespace='/test')
        #return ""
    return render_template('index.html')


@socketio.on('connect', namespace='/test')
def test_connect():
    # need visibility of the global thread object
    global thread
    print('Client connected')

    #Start the random number generator thread only if the thread has not been started before.
    if not thread.isAlive():
        print("Checking Emails")
        thread = socketio.start_background_task(readIr.check_for_email(pan_mail))

@socketio.on('disconnect', namespace='/test')
def test_disconnect():
    print('Client disconnected')

if __name__ == "__main__":
    socketio.run(app)