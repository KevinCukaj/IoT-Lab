# Il seguente codice crea un publisher e subscriber per due topic diversi:
#   - cmd_vel:      topic su cui pubblichiamo per dire al drone di salire in altitudine
#   - odometry:     topic in cui ascoltiamo per ottenere l'altitudine in tempo reale del drone



import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

global TARGET_POINT             # the target point the drone needs to be oriented towards
TARGET_POINT =  [8,-2]
global TARGET_HEIGHT            # the target height the drone has to reach
TARGET_HEIGHT = 4.0
global CORRECT_ROTATION
CORRECT_ROTATION = False


class MyDrone(Node):

    def __init__(self):
        super().__init__('gazebo_publisher')
        self.drone_position = 0     # variable for containing the current height of the drone
        self.drone_orientation = 0  # variable for containing the current orientation of the drone
        self.drone_posX = 0
        self.drone_posY = 0
        self.publisherTwist = self.create_publisher(        
            Twist,                  # type of message the topic takes
            'cmd_vel',              # name of the topic
            10)
        
        self.subscriberOdom = self.create_subscription(
            Odometry,               # type of message the topic takes
            'odometry',             # name of the topic
            self.store_position,    # the function that will be executed when messages are received by our subscriber.
            10)                     # length of the queue of the subscriber, it means that if at a given time we have a queue of more than 10 published messages, additional messages will be dropped.

        self.stop_msg = Twist()
        self.velocity = Twist()
        self.timer = self.create_timer(0.5, self.publish_movement)
        self.rotation_increment = math.pi/20        # the rate in which the drone turns


    def publish_movement(self):
        global CORRECT_ROTATION 
        #self.get_logger().info(f"storing position {self.drone_posX, self.drone_posY}") 
        #self.get_logger().info(f"storing orientation {self.drone_orientation}")
        self.get_logger().info(f"remaining distance {find_distance(TARGET_POINT[0], TARGET_POINT[1], self.drone_posX,self.drone_posY)}")
        if abs(TARGET_HEIGHT-self.drone_position)>0.001:                    # check if the drone is close enough to the target point
            self.velocity.linear.z = (TARGET_HEIGHT-self.drone_position)/2  # dynamically adjust speed
            self.publisherTwist.publish(self.velocity)                      # publish the message
        else:
            self.velocity.linear.z = 0.0                                    # stop changing altitude
            self.publisherTwist.publish(self.velocity)



        if abs(self.drone_orientation-find_angle(TARGET_POINT[0],TARGET_POINT[1])[0]) > 0.2:
            # depending where is the point, the drone will either rotate clockwise or counter-clockwise to obtain the correct orientation the fastest
            self.velocity.angular.z = self.rotation_increment * find_angle(TARGET_POINT[0],TARGET_POINT[1])[1]
            self.publisherTwist.publish(self.velocity)          # publish the message
        else: 
            self.velocity.angular.z = 0.0                       # stop rotating
            self.publisherTwist.publish(self.velocity)
            #self.publisherTwist.publish(self.stop_msg)  # publish an empty message to stop the drone
            CORRECT_ROTATION = True

        if abs(find_distance(TARGET_POINT[0], TARGET_POINT[1], self.drone_posX,self.drone_posY))>0.1 and CORRECT_ROTATION == True:
            self.velocity.linear.x = (find_distance(TARGET_POINT[0], TARGET_POINT[1], self.drone_posX, self.drone_posY)/3)
            self.publisherTwist.publish(self.velocity)
        else:
            CORRECT_ROTATION = None
            self.velocity.linear.x = 0.0                                    # stop changing altitude
            self.publisherTwist.publish(self.velocity)
    
    def store_position(self, odometry_msg : Odometry):
        x = odometry_msg.pose.pose.orientation.x
        y = odometry_msg.pose.pose.orientation.y
        z = odometry_msg.pose.pose.orientation.z
        w = odometry_msg.pose.pose.orientation.w
        self.drone_position = odometry_msg.pose.pose.position.z                 # store the current height of the drone 
        self.drone_posX = odometry_msg.pose.pose.position.x
        self.drone_posY = odometry_msg.pose.pose.position.y
        self.drone_orientation = euler_from_quaternion(x,y,z,w)[2]
        #self.get_logger().info(f"storing position {self.drone_position}")       # (optional) print on the console
        #self.get_logger().info(f"storing orientation {self.drone_orientation}") # (optional) print on the console
        


def euler_from_quaternion(x, y, z, w):
    
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +1.0 - 2.0 * (y * y + z * z)
    t4 = +2.0 * (w * z + x * y)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians



def find_angle(x, y):
    absX = abs(x)
    absY = abs(y)

    if x>0 and y>0:     # ok
        s = 1
        theta = math.pi/2 
        theta -= math.atan(absY/absX)
    elif x>0 and y<0:   # ok
        s = -1
        theta = math.pi/2 
        theta += math.atan(absY/absX)
    elif x<0 and y<0:   # ok
        s = -1
        theta = -math.pi/2
        theta -= math.atan(absY/absX)
    elif x<0 and y>0:   # ok
        s = 1
        theta = -math.pi/2
        theta += math.atan(absY/absX)
    return theta, s

def find_distance(x,y, xNow, yNow):
    return math.sqrt((x-xNow)**2+(y-yNow)**2)


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
