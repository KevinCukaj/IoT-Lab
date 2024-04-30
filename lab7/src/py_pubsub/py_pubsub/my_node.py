# Il seguente codice crea un publisher e subscriber per due topic diversi:
#   - cmd_vel:      topic su cui pubblichiamo per dire al drone di salire in altitudine
#   - odometry:     topic in cui ascoltiamo per ottenere l'altitudine in tempo reale del drone



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class MyDrone(Node):

    def __init__(self):
        super().__init__('gazebo_publisher')

        self.drone_position = 0     # variable for containing the current height of the drone

        self.publisherTwist = self.create_publisher(        
            Twist,                  # type of message the topic takes
            'cmd_vel',              # name of the topic
            10)
        
        self.subscriberOdom = self.create_subscription(
            Odometry,               # type of message the topic takes
            'odometry',             # name of the topic
            self.store_position,    # the function that will be executed when messages are received by our subscriber.
            10)                     # length of the queue of the subscriber, it means that if at a given time we have a queue of more than 10 published messages, additional messages will be dropped.
        
        self.timer = self.create_timer(1.0, self.publish_velocity)
        
        self.stop_msg = Twist()
        self.velocity = Twist()
        
        self.direction = 0

    def publish_velocity(self):
        if self.drone_position < 4:                     # check if the current position of the drone is lower than a given value
            self.velocity.linear.z = 1.0                # go up
            self.publisherTwist.publish(self.velocity)  # publish the message
        else:
            self.publisherTwist.publish(self.stop_msg)  # publish an empty message to stop the drone
    
    def store_position(self, odometry_msg : Odometry):
        self.drone_position = odometry_msg.pose.pose.position.z             # store the current height of the drone 
        self.get_logger().info(f"storing position {self.drone_position}")   # (optional) print on the console
        

def main(args=None):
    rclpy.init(args=args)

    drone = MyDrone()

    executor = MultiThreadedExecutor()
    executor.add_node(drone)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
