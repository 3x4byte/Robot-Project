from mqtt_client import MqttClient
from vector import Vector


class Robot:

    vector = Vector()

    def __init__(self, position=None, rotation=None, pixel_per_step=None):
        self.position = position
        self.rotation = rotation
        self.pixel_per_step = pixel_per_step
        self.degrees_per_step = 1/2.95
        self.mqtt_client = MqttClient("robot")

    def init(self):
        self.mqtt_client.send_init_msg()

    def reference_move(self, steps, speed):
        self.mqtt_client.send_msg("movement", "forward", steps, speed)

    def move(self, steps, speed=500):
        self.mqtt_client.send_msg("movement", "forward", steps, speed)
        self.position = self.position + (steps * self.pixel_per_step)*self.vector.unit(self.rotation)

    def rotate(self, steps, speed=300):
        self.mqtt_client.send_msg("movement", "right", steps, speed)
        self.rotation = self.vector.rotate(self.rotation, (steps*self.degrees_per_step))

    def grab(self, steps, speed=500):
        self.mqtt_client.send_msg("movement", "grab", steps, speed)

    def release(self, steps, speed=500):
        self.mqtt_client.send_msg("movement", "release", steps, speed)

    def up(self, steps, speed=500):
        self.mqtt_client.send_msg("movement", "up", steps, speed)

    def down(self, steps, speed=500):
        self.mqtt_client.send_msg("movement", "down", steps, speed)

    def brick_delivered(self):
        self.mqtt_client.send_delivered_msg()
