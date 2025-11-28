#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String # < import msg type

class RobotNewsStation(Node): 
    def __init__(self):
        super().__init__("robot_news_station") # node name (same as file name)
        self.declare_parameter("robot_name", "C3P0")
        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        # String is the msg type, robot_news is the topic name, 10 is the queue size (buffer)
        # The msg type is a parameter for topics
        # There are some existing msg types
        # String is one of them
        # can be checked in cli:
        # > cd
        # > ros2 interface show example_interfaces/msg/String
        # which returns "string data"
        # in the msg folder, lots of existing msg types
        # we have to import them, see top of the file
        # ANd also modify the dependency in package.xml

        self.create_timer(0.5, self.publish_news) 
        # create a timer to call the publish_news function every 0.5 second  

        self.get_logger().info("Robot News Station is online!")
        # To show something when the node is started

    def publish_news(self):
        msg = String() # create a msg object
        msg.data = "Hi, this is " + self.robot_name_ + " from the robot_news_station" # fill in the data field
        self.publisher_.publish(msg) # publish the msg
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()