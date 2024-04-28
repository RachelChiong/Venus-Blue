"""
mqtt_thread.py

Semester 1, 2024
CSSE4011
"""

from threading import Thread
import time
import json

import paho.mqtt.client as mqtt
import numpy as np

__author__ = "Rachel Chiong"

Data_Packet = "{\"distance\": %f}"
data = ""
HOST = "csse4011-iot.zones.eait.uq.edu.au"
MQTT_TOPIC = "46980388"

class MqttThread(Thread):
    """
    MQTT Thread which handles the MQTT communications (subscription and
    retrieval of data)

    Reference:
        paho_mqtt_sub.py from CSSE4011 Prac 3 example code
    """
    def __init__(self, name, hostname, topic):
        Thread.__init__(self)
        self.name = name
        self.hostname = hostname
        self.topic = topic
        self.payload = ""
        self.counter = 0
        # Initialise the thread
        try:
            self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.mqttc.on_connect = self.on_connect
            self.mqttc.on_message = self.on_message
            self.mqttc.on_subscribe = self.on_subscribe
            self.mqttc.on_unsubscribe = self.on_unsubscribe
            self.mqttc.connect(self.hostname)
            print("Is connected: ", self.mqttc.is_connected())
        except:
            print("Issue initialising so skipping...")
        print("MQTT client initialised")
        self.get_payload()

    def get_payload(self):
        """
        Gets the distance payload from the ultrasonic sensor via MQTT
        #TODO: Update this to process Turtlebot and periperhal payloads

        Returns:
            payload (str): JSON packet containing the distance
        """
        if self.payload == "":
            payload = f'{{\"distance\": {self.counter}}}'
            self.counter = self.counter + 1
            return json.loads(payload)
        return self.payload

    def run(self):
        print("Running loop...")
        self.client_loop()

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
        print("subscribing...")
        if reason_code_list[0].is_failure:
            print(f"Broker rejected you subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        # Be careful, the reason_code_list is only present in MQTTv5.
        # In MQTTv3 it will always be empty
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()

    def on_message(self, client, userdata, message):
        """
        Callback function which processes the retrieved message. It retrieves
        the distance from the MQTT packet, sets the payload variable and updates
        the kalman filter.

        #TODO: Update this
        """
        payload = message.payload.decode("utf-8")  # Decode payload as text
        try:
            self.payload = json.loads(payload)  # Load JSON data
            distance = self.payload['distance'] / 100  # Access the 'distance' value and convert to meters
            kalman.ultrasonic.update_kalman(distance) # update kalman filter with distance
        except json.JSONDecodeError:
            print("Received invalid JSON payload.")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        """
        Callback function which occurs when a connection request is made
        """
        print("Connecting...")
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            client.subscribe(self.topic)

    def client_loop(self):
        """
        Client loop which constantly listens for MQTT messages
        """
        print("Starting loop...")
        self.mqttc.loop_forever()

if __name__ == "__main__":
    a = MqttThread("myThread_name_A", HOST, MQTT_TOPIC)
    a.start()
