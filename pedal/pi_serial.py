import serial
from pi_mqtt import mqtt_publish
import json
from threading import Thread

# Serial connection to base node
# baud rates above ~50000 seem to occassionally error when large volumes of
# data is sent for some reason
SERIAL_DEVICE = '/dev/ttyACM0'

BAUD_RATE = 115200
base_node_serial = serial.Serial(SERIAL_DEVICE, BAUD_RATE, timeout=None)

class NucleoConnection(Thread):
    def __init__(self, serial: serial.Serial,
                  *args, **kwargs) -> None:
        self.serial = serial
        super().__init__(*args, **kwargs)

    def run(self) -> None:
        while True:
            line = self.serial.readline().decode()
            print("Received:", line)
            try:
                json.loads(line)
            except json.JSONDecodeError as e:
                print(f"Invalid JSON received from nucleo: {e}")
            else:
                mqtt_publish(line.strip('\n'))

if __name__ == "__main__":
    comms = NucleoConnection(base_node_serial)
    comms.start()