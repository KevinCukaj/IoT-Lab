import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import SetPen

from random import randint

import math


turtle_moves = [
    Vector3(x=2.0, y=0.0, z=0.0),
    Vector3(x=0.0, y=0.0, z=0.0),
]
turtle_turns = [
    Vector3(x=0.0, y=0.0, z=0.0),
    Vector3(x=0.0, y=0.0, z=math.pi/2)
]

class RainbowClient(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher = self.create_publisher( # create a publisher for a topic (Topics are named buses over which nodes exchange messages)
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.client = self.create_client(SetPen, '/turtle1/set_pen')    # create a new service client
        self.req = SetPen.Request()

        self.timer = self.create_timer(1, self.timer_callback)
        self.i = 0  

    def timer_callback(self):
        msg = Twist()
        msg.linear = turtle_moves[self.i%2]
        msg.angular = turtle_turns[self.i%2]

        self.req.r = randint(0, 255)
        self.req.g = randint(0, 255)
        self.req.b = randint(0, 255)
        self.req.width = 10
        self.req.off = 0

        self.future = self.client.call_async(self.req)
        self.publisher.publish(msg)

        self.i += 1


def main(args=None):
    rclpy.init(args = args)

    rainbow_client = RainbowClient()

    rclpy.spin(rainbow_client)

    rainbow_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
