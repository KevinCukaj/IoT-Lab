import sys

from tutorial_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # Create a new service client.
        while not self.cli.wait_for_service(timeout_sec=1.0):                       # we are telling our node to wait for the service to be available before continuing with the execution.
            self.get_logger().info('service not available, waiting again...')       #
        self.req = AddThreeInts.Request()  # we instantiate our request message by using the constructor Request() of our AddTwoInts interface.

    def send_request(self, a, b, c):       # Here, we are configuring our request and then using it in the function call_async().
        self.req.a = a
        self.req.b = b
        self.req.c = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    minimal_client.get_logger().info(
        'Result of add_three_ints: for %d + %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		