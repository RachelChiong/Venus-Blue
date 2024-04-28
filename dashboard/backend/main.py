"""
Main.py
Prac 3 for PC-GUI - Flask backend for PC-GUI
Semester 1, 2024
CSSE4011
"""

import json
from flask import Flask, request
from flask_cors import CORS, cross_origin
from mqtt_thread import MqttThread

__author__ = "Rachel Chiong"

HOST = "csse4011-iot.zones.eait.uq.edu.au"
MQTT_TOPIC = "TOPIC"

app = Flask(__name__)

# Handle CORS as necessary
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

@app.route('/')
@cross_origin
def root():
    return 'Hello from Flask backend!'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)