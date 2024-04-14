import sys

from tutorial_interfaces.srv import TurnTurtle
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(TurnTurtle, 'turn_turtle')       # Create a new service client.
        while not self.cli.wait_for_service(timeout_sec=1.0):                       # we are telling our node to wait for the service to be available before continuing with the execution.
            self.get_logger().info('service not available, waiting again...')       #
        self.req = TurnTurtle.Request()  # we instantiate our request message by using the constructor Request() of our AddTwoInts interface.

    def send_request(self, rotation):       # Here, we are configuring our request and then using it in the function call_async().
        self.req.rotation = rotation

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]))
    minimal_client.get_logger().info('Done rotating turtle')
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		