from tutorial_interfaces.srv import AddThreeInts # we are importing an example interface called AddTwoInts from the example_interfaces package

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback) # Create a new service client. create_service() is a Node method

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info('Incoming request\na: %d b: %d c:%d' % (request.a, request.b, request.c))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()