# MQTT Host
HOST = "csse4011-iot.zones.eait.uq.edu.au"

TB32_IP = "192.168.0.32"
TB39_IP = "192.168.0.39"

# template topic
MQTT_TOPIC = "topic"

FOOTPEDAL_TOPIC = "venusBlueFootPedal"
LOCALISATION_TOPIC = "venusBlueLocalisation"
TELEMETRY_TOPIC = "venusBlueTelemetry"

# Packet structure
FOOTPEDAL_PACKET = {
    "x": 0,
    "y": 0,
    "z": 0
}

TELEMETRY_PACKET = {
    "vx": 0,
    "vy": 0
}

LOCALISATION_PACKET = {
    "x": 0,
    "y": 0
}