from tutorial_interfaces.srv import TurnTurtle # we are importing an example interface called TurnTurtle from the example_interfaces package

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import math

class Turtle(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher = self.create_publisher( # create a publisher for a topic (Topics are named buses over which nodes exchange messages)
            Twist,
            '/turtle1/cmd_vel',
            10
        )
    
    def spin(self):
        msg = Twist()
        #msg.linear = Vector3(x=2.0, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=(math.pi/(2*r)))

        self.publisher.publish(msg)

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(TurnTurtle, 'turn_turtle', self.turn_turtle_callback) # Create a new service client. create_service() is a Node method
        self.turtle = Turtle()

    def turn_turtle_callback(self, request, response):
        global r
        r = 0
        if request.rotation%2==0:
            r=-1
            self.get_logger().info('Spinning the turtle clockwise ...')
        else:
            r=1
            self.get_logger().info('Spinning the turtle counter-clockwise ...')
            
        self.turtle.spin()  # Assuming your Turtle class has a spin method
        response.y = 1
        self.get_logger().info('Done spinning the turtle.')
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
