import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
import math


'''
This program create the node that controls the movement of turtle1.
'''


class RandSubscriber(Node):


    def __init__(self,name):

        super().__init__(name)

        self.turtlename = self.declare_parameter(
            'turtlename','turtle1').get_parameter_value().string_value 
        
        # The variable turtle1's linear speed. Can be modified. Must be float, rather than integers.
        self.VELO = 3.0

        # Initialize turtle1's current position, target position, motion.
        self.currentPose = Pose(x=5.544445,y=5.544445,theta = 0.000000)
        self.targetPose = self.currentPose
        v3 = Vector3(x=0.0,y=0.0,z=0.0)
        self.twist = Twist(linear = v3, angular =v3)

        # Subscriptions and publishes.
        # Subscribe to the position of turtle1, and the position of the target (i.e. turtle2), so as to calculate its next movement.
        self.position_subscriber = self.create_subscription(Pose,'/turtle1/pose',self.updPose_callback,10)
        self.goal_pose_subscriber = self.create_subscription(Pose,'/turtle2/pose',self.updTarget_callback,10) 
        # Publish a command to turtle1, so as to control its movement.
        self.velocity_publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)


    def updPose_callback(self,pose):
        # Constantly update turtle1's current position.
        self.currentPose = pose


    def updTarget_callback(self,pose):
        # When a new target is published, update the target, calculate and send the command for the next movement.
        self.targetPose = pose
        self.move_callback()


    def move_callback(self):

        dx = self.targetPose.x - self.currentPose.x
        dy = self.targetPose.y - self.currentPose.y

        rad = math.atan2(dy,dx) # rad is a value in radians, rather than degrees. range: [-pi, pi)
        vx = self.VELO*math.cos(rad)
        vy = self.VELO*math.sin(rad)

        # Publish the calculated next movement, to control turtle1.
        self.twist = Twist(linear=Vector3(x=vx,y=vy,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        self.velocity_publisher_.publish(self.twist)



def main(args=None):
    rclpy.init(args=args)
    # Initialize the node that controls turtle1.
    ctrl_node = RandSubscriber('turtle1_ctrl')
    rclpy.spin(ctrl_node)
    rclpy.shutdown()