import argparse
import asyncio
import paho.mqtt.client as mqtt

async def main(server, port, topic, rate, message):

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.suppress_exceptions = True

    @client.connect_callback()
    def connect_callback(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f'failed to connect: {reason_code}')
            return

        print('connected')
        client.subscribe(self._mqtt_topic)

    @client.disconnect_callback()
    def disconnect_callback(client, userdata):
        print(f'disconnected')

    client.loop_start()

    # Connect to mqtt
    while True:

        try:
            error = client.connect(host = server, port = port)
        except Exception as err:
            print(err)
            await asyncio.sleep(1)
            continue

        if error:
            print(mqtt.error_string(error))
            await asyncio.sleep(1)
            continue

        while True:
            client.publish(topic, message)
            await asyncio.sleep(1 / rate)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--server',
        type = str,
        default = 'localhost'
    )

    parser.add_argument(
        '--port',
        type = int,
        default = 1883
    )

    parser.add_argument(
        '--topic',
        type = str,
        default = 'us45286524'
    )

    parser.add_argument(
        '--rate',
        type = str,
        default = 1
    )

    parser.add_argument(
        '--message',
        type = str,
        default = '{"x": 1.0, "y": 1.0, "z": 1.0}'
    )

    args = parser.parse_args()

    asyncio.run(
        main(
            args.server,
            args.port,
            args.topic,
            args.rate,
            args.message
        )
    )
