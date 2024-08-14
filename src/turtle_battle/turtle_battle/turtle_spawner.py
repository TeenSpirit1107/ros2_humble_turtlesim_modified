import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.msg import Pose
from functools import partial
import random,math
from time import time


'''
This program create the node that spawns and kills turtle2.
'''


class TurtleSpawner(Node):


    def __init__(self) -> None:

        # Initialize the name and position of turtle2
        super().__init__('turtle_spawner')
        self.declare_parameter("object_name","turtle2")
        self.turtle_name = self.get_parameter("object_name").value
        self.object_name = "turtle2_obj"
        self.tur1_pose = None
        # This counts the number that turtle2 is eaten.
        self.counter = 0 
        # This variable can be modified. The maximum distance between turtle1 and turtle2 when they are judged as coincide.
        self.DIST_ACCURACY = 0.5 

        # Subscribe to the position of turtle1, to judge whether turtle2 is eaten.
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.tur1_pose_callback, 10)
        
        # Spawn a new turtle.
        self.spawn_new_turtle()
        self.tur2_pose = Pose(x=0.0, y=0.0, theta = 0.0)
        # Every 0.1 seconds, check whether it's reached by turtle2.
        self.control_loop_timer_ = self.create_timer(0.1, self.checkReached_callback)
        self.tick = 0
    
    def spawn_new_turtle(self):
        # Generate the random posision of the new turtle2, and call t

        name = self.turtle_name
        tar_x = random.uniform(0.5, 10.5) # This can be modified. x, y's position's range is approximately [0, 11]
        tar_y = random.uniform(0.5, 10.5)
        tar_theta = random.uniform(-math.pi, math.pi)
        self.get_logger().info("Another turtle is spawned at (%f,%f), with angle %f."%(tar_x,tar_y,tar_theta))

        # Request a service: spawn a new turtle with this generated position and angle.
        self.call_spawn_server(name, tar_x, tar_y, tar_theta)
    

    def call_spawn_server(self, turtle_name, x, y, theta):
        # This is called after a random position of turtle2 is generated. It calls the server to spawn a new turtle2 at the given position.
        client = self.create_client(Spawn, "spawn") 
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        # When generating is done.
        future.add_done_callback(
            partial(self.callback_call_spawn, x=x, y=y, theta=theta))
    
    
    def tur1_pose_callback(self, pose):
        # Constantly updated about turtle1's position.
        self.tur1_pose_ = pose

    
    def checkReached_callback(self):
        # Check whether turtle2 is reached by turtle1.
        
        # If turtle1 has not been initialized, the function returns, to avoid causing exceptions.
        if self.tur1_pose_ == None :
            return
        
        # Calculate the distance between turtle1 and turtle2, to judge whether turtle1 is reached. 
        dist_x = self.tur2_pose.x - self.tur1_pose_.x
        dist_y = self.tur2_pose.y - self.tur1_pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        # If their distance is close enough, they are judged as coinciding with each other.
        # Kill the current turtle2. A new turtle2 is spawned by function "call_kill_server".
        if distance < self.DIST_ACCURACY:  
            self.call_kill_server(self.turtle_name)
            self.counter += 1
            self.get_logger().info("Goal reached %d times."%self.counter)

    
    def call_kill_server(self, turtle_name):
        # This is called when turtle2 is reached by turtle1. It calls the server to kill turtle2, and then calls a function that spawn a new turtle2.
        
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Kill.Request()
        request.name = turtle_name
        future = client.call_async(request)
        # The function "call_back_call_kill" calls the spawn of a new turtle.
        future.add_done_callback(
            partial(self.callback_call_kill, ))
    
    def callback_call_kill(self, future):
    # It's called when turtle2 is killed.
        try:
            future.result()
            self.spawn_new_turtle()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def callback_call_spawn(self, future, x, y, theta):
        # This is called after the server spawn a new turtle2 at the given random position.
        # It updates turtle2's position.
        # And turtle2's position is not updated constantly, but only updated after it's created. This is sufficient becaues turtle2 doesn't move.
        try:
            response = future.result()
            self.tur2_pose = Pose(x=x,y=y,theta=theta)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        


def main(args=None):
    rclpy.init(args=None)
    # Initialize the node that spawns turtle 2.
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()