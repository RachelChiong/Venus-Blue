# MQTT Host
HOST = "csse4011-iot.zones.eait.uq.edu.au"

# template topic
MQTT_TOPIC = "topic"

FOOTPEDAL_TOPIC = "vb-footpedal"
LOCALISATION_TOPIC = "vb-localisation"
TELEMETRY_TOPIC = "vb-telemetry"

# Packet structure
FOOTPEDAL_PACKET = {
    "x": 0,
    "y": 0,
    "z": 0
}

TELEMETRY_PACKET = {
    "velocity": 0
}

LOCALISATION_PACKET = {
    "x": 0,
    "y": 0
}