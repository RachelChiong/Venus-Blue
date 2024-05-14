import paho.mqtt.client as mqtt

HOST = "csse4011-iot.zones.eait.uq.edu.au"#"mqtt.eclipseprojects.io"#"csse4011-iot.zones.eait.uq.edu.au"
MQTT_TOPIC = "venusBlueFootPedal"#"paho/test/topic"#"47443732"

def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    pass

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_publish = on_publish

mqttc.connect(HOST)
mqttc.loop_start()

def mqtt_publish(message: str):
    return mqttc.publish(MQTT_TOPIC, message, qos=1).wait_for_publish()

#mqtt_publish('{"x":1, "y":42, "z":93}').wait_for_publish()