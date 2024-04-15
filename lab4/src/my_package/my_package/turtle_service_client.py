import sys

import rclpy
from rclpy.node import Node

from tutorial_interfaces.srv import SpinTurtle # import the interface called SpinTurtle from the tutorial_interfaces package
# In this file, such interface is used to define a service client.


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        
        self.cli = self.create_client(SpinTurtle, 'spin_turtle')       # Create a new service client.
        
        while not self.cli.wait_for_service(timeout_sec=1.0):                       # we are telling our node to wait for the service to be available before continuing with the execution.
            self.get_logger().info('Service not available, waiting again...')       #
        self.req = SpinTurtle.Request()  # we instantiate our request message by using the constructor Request() of our SpinTurtle interface.

    def send_request(self, direction):       # Here, we are configuring our request and then using it in the function call_async().
        self.req.direction = direction
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]))    # get the response of the service server
    minimal_client.get_logger().info(response.result)           # print the response of the service server
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		