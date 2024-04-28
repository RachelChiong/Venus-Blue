"""
Main.py
Prac 3 for PC-GUI - Flask backend for PC-GUI
Semester 1, 2024
CSSE4011
"""

import json
from flask import Flask, request
from flask_cors import CORS, cross_origin

__author__ = "Rachel Chiong"

HOST = "csse4011-iot.zones.eait.uq.edu.au"
MQTT_TOPIC = "TOPIC"

app = Flask(__name__)

@app.route('/')
def hello():
    return 'Hello from Flask backend!'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)