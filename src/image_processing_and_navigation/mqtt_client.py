import time
import paho.mqtt.client as mqtt
import ast


class MqttClient:

    def __init__(self, name):
        self.client = mqtt.Client(client_id=name,
                             transport="tcp",
                             protocol=mqtt.MQTTv311)
        self.client.connect("127.0.0.1", port=1883, keepalive=60)
        time.sleep(0.5)
        self.client.loop_start()

    def send_init_msg(self):
        message = {
            "type": "initializasion"
        }
        self.client.publish("robots/collector", str(message))

    def send_pause_msg(self, pause):
        message = {
            "type": "pause",
            "value": pause
        }
        self.client.publish("robots/collector", str(message))

    def send_pause1_msg(self):
        message = {
            "type": 2,
            "info": 3
        }
        self.client.publish("robots/sorter", str(message))

    def send_msg(self, type, operation, steps, speed):
        message = {
            "type": type,
            "operation": operation,
            "steps": steps,
            "speed": speed
        }
        self.client.publish("robots/collector", str(message))

    def send_delivered_msg(self):
        message = {
            "type": "information",
            "value": "brick_delivered"
        }
        self.client.publish("robots/sorter", str(message))

    def subscribe_to(self, topic):
        self.client.subscribe(topic)

    def on_message(self, func):
        self.client.on_message = func

    def parse_message(self, message_object):
        return ast.literal_eval(message_object.payload.decode("UTF-8"))

