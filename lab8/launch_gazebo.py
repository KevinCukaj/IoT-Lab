from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from random import randint

WORLD_FILE = "multicopter.sdf"

def generate_launch_description():

  return LaunchDescription(
    [
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              FindPackageShare('ros_gz_sim'),
              'launch/'
              'gz_sim.launch.py',
          ])
        ]),
        launch_arguments={
          'gz_args' : "./"+WORLD_FILE
        }.items()
      ),


      #### We create two bridges (through Node objects) ... one for each topic #### 
      
      # In case we deal with more than one drone (in this case, the drone is called X3)
      # we remove the "/X3/" at the beginning when writing down the arguments. Then,
      # on the my_node.py file, we define which drone we are refering to through when 
      # we definie a publisher or a subscriber. 
      # E.g.: self.subscriberOdom = self.create_subscription(Odometry,'{DRONE_NAME}/odometry', self.store_position, 10)

      Node(
        package='ros_gz_bridge',
        name='cmd_vel',
        executable='parameter_bridge',
        arguments=
          [
          "/X3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
          ]
      ),

      Node(
        package='ros_gz_bridge',
        name='odometry',
        executable='parameter_bridge',
        arguments=
          [
          "/X3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
          ]
      ),

      Node(
        package='py_pubsub',
        namespace='X3',
        executable='my_node',
        name='sim'
        ),
    ]
  )


