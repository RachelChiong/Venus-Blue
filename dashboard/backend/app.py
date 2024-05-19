"""
Main.py
Venus-Blue PC-GUI - Flask backend for PC-GUI
Semester 1, 2024
CSSE4011
"""

import json
from flask import Flask, request
from flask_cors import CORS, cross_origin
from mqtt_thread import MqttThread

# Import all common constants
from common import *

__author__ = "Rachel Chiong"

app = Flask(__name__)

# Handle CORS as necessary
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

# Create thread stubs
Th_Footpedal = MqttThread("footpedal", HOST, MQTT_TOPIC)
Th_Localisation = MqttThread("localisation", HOST, MQTT_TOPIC)
Th_Telemetry = MqttThread("telemetry", HOST, MQTT_TOPIC)

@app.route('/')
@cross_origin()
def root():
    if not Th_Footpedal.isConnected:
        Th_Footpedal.config(HOST, FOOTPEDAL_TOPIC)
        Th_Localisation.config(HOST, LOCALISATION_TOPIC)
        Th_Telemetry.config(HOST, TELEMETRY_TOPIC)

        Th_Footpedal.start()
        Th_Localisation.start()
        Th_Telemetry.start()

    return 'Hello from Flask backend!'

@app.route('/connect')
@cross_origin()
def default_connect():
    if Th_Footpedal.isConnected:
        return
    Th_Footpedal.config(HOST, FOOTPEDAL_TOPIC)
    Th_Localisation.config(HOST, LOCALISATION_TOPIC)
    Th_Telemetry.config(HOST, TELEMETRY_TOPIC)

    Th_Footpedal.start()
    Th_Localisation.start()
    Th_Telemetry.start()
    return "Connected"

@app.route('/telemetry')
@cross_origin()
def get_telemetry():
    return Th_Telemetry.get_payload()

@app.route('/footpedal')
@cross_origin()
def get_footpedal():
    payload = Th_Footpedal.get_payload()
    print(payload)
    return payload

@app.route('/localisation')
@cross_origin()
def get_localisation():
    return Th_Localisation.get_payload()

if __name__ == '__main__':
    app.run(port=5001)
