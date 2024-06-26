import os
import json
import math
import asyncio
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

class VenusBlue(Node):

    def __init__(
            self,
            server = 'csse4011-iot.zones.eait.uq.edu.au',
            port = 1883
        ):
        super().__init__('venusblue')

        # Mqtt client to get messages from.
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.xPos = 0
        self.yPos = 0

        # Declare parameters
        self.declare_parameter('mqtt_server', 'csse4011-iot.zones.eait.uq.edu.au')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('pedal_topic', 'venusBlueFootPedal')
        self.declare_parameter('telementry_topic', 'venusBlueTelemetry')
        self.declare_parameter('motor_topic', 'cmd_vel')

        try:
            ros_domain = os.environ["ROS_DOMAIN_ID"]
            ros_hostname = os.environ["ROS_HOSTNAME"]
            ros_uri = os.environ["ROS_MASTER_URI"]
        except:
            ros_domain = 'unknown'
            ros_hostname = 'unknown'
            ros_uri = 'unknown'

        self.get_logger().info(
            '\n' + '\n'.join([
                'parameters:',
                f'    ros_hostname: {ros_hostname}',
                f'    ros_uri: {ros_uri}',
                f'    ros_domain: {ros_domain}',
                f'    mqtt_server: {self.get_parameter("mqtt_server").value}',
                f'    mqtt_port: {self.get_parameter("mqtt_port").value}',
                f'    pedal_topic: {self.get_parameter("pedal_topic").value}',
                f'    motor_topic: {self.get_parameter("motor_topic").value}',
            ])
        )

        # Messages from mqtt to handle.
        self._messages = asyncio.Queue()  


    async def mqtt_task(self):

        self._client.suppress_exceptions = True

        @self._client.connect_callback()
        def connect_callback(client, userdata, flags, reason_code, properties):
            if reason_code.is_failure:
                self.get_logger().error(f'failed to connect: {reason_code}')
                return

            self.get_logger().info('mqtt connected')

            # Subscribe to the pedal topic.
            try:
                pedal = self.get_parameter("pedal_topic").value
                client.subscribe(pedal)
            except Exception as err:
                self.get_logger().error(f'failed to subscribe: {err}')

        @self._client.disconnect_callback()
        def disconnect_callback(client, userdata):
            self.get_logger().info(f'mqtt disconnected')

        @self._client.subscribe_callback()
        def subscribe_callback(client, userdata, mid, reason_code_list, properties):
            """Callback on subscribing to a mqtt topic."""
            if reason_code_list[0].is_failure:
                self.get_logger().error(f"mqtt subscription failed: {reason_code_list[0]}")
            else:
                self.get_logger().info(f"mqtt subscribed")

        @self._client.unsubscribe_callback()
        def unsubscribe_callback(client, userdata, mid, reason_code_list, properties):
            """Callback on unsubscribing to a mqtt topic."""
            if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
                self.get_logger().info("mqtt unsubscribed")
            else:
                self.get_logger().error(f"mqtt failed to unsubscribe: {reason_code_list[0]}")

        @self._client.message_callback()
        def message_callback(client, userdata, message: mqtt.MQTTMessage):
            """Push a message received over mqtt to the message queue"""

            try:
                parsed = json.loads(message.payload)
                message = (
                    int(parsed['x']),
                    int(parsed['y']),
                    int(parsed['z'])
                )
            except Exception as err:
                self.get_logger().error(f'Failed to parse json {message.payload}: {err}')
                return

            try:
                self._messages.put_nowait(message)
            except Exception as err:
                self.get_logger().error(f'Message queue full. Dropping message.')

        self._client.loop_start()

        # Connect to mqtt
        while True:
            server = self.get_parameter('mqtt_server').value
            port = self.get_parameter('mqtt_port').value
            try:
                self.get_logger().info(f'mqtt connect to {server} at {port}')
                error = self._client.connect(host = server, port = port)
            except Exception as err:
                self.get_logger().error(f'{err}')
                await asyncio.sleep(1)
                continue

            if error:
                self.get_logger().error(mqtt.error_string(error))
                await asyncio.sleep(1)
                continue

            while True:
                self._messages.put_nowait((0, 0, 0))
                await asyncio.sleep(1)

    async def tester(self):
        while rclpy.ok():
            self._messages.put_nowait((1, 1, 0))
            await asyncio.sleep(1)

    async def ros_update(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 0.1)
            await asyncio.sleep(0.1)

    async def ros_task(self):

        motor_topic = self.get_parameter('motor_topic').value
        telementry_topic = self.get_parameter('telementry_topic').value
        commander = self.create_publisher(Twist, motor_topic, 10)
        self.get_parameter('motor_topic').value

        self.target_frame = 'map'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        while rclpy.ok():

            linear_max = 0.18 # 0.26
            angular_max = 0.80 # 1.82

            # Wait for a command message.
            x, y, z = await self._messages.get()
            x = min (x, 230)
            y = min (y, 230)

            # Get left and right linear velocities.
            vx = float(((x & 0x7F) / 128.0) * linear_max)
            vy = float(((y & 0x3f) / 64) * linear_max)

            # Total velocity is magnitude of both.
            velocity = min(math.sqrt(vx ** 2 + vy ** 2), linear_max)

            # Angular is the difference between the velocities.
            angular = min((vy - vx) / (2 * linear_max) * angular_max, angular_max)
            
            twist = Twist()
            

            if (z < 45):
                velocity = -velocity
                angular = -angular

            twist.linear.x = velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular

            self._client.publish(
                telementry_topic,
                json.dumps({"vx": vx, "vy": vy, "linear":velocity, "angular": angular, "xLocation": self.xPos, "yLocation": self.yPos})
            )

            self.get_logger().info(f'send vx: {vx}, vy: {vy}, velocity: {velocity}, angular: {angular}')
            commander.publish(twist)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base_footprint'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        self.xPos = t.transform.translation.x
        self.yPos = t.transform.translation.y

    async def main(self):
        await asyncio.gather(
            self.mqtt_task(),
            self.ros_task(),
            self.ros_update()
        )

def main():
    rclpy.init()
    node = VenusBlue()
    asyncio.run(node.main())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
