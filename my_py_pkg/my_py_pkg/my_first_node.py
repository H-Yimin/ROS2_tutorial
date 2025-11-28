#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# ////Minimal Code////////////////////////////
# def main(args=None):
#     # Initialize the ros2 (communication,etc.) with arguments from main
#     rclpy.init(args=args)

#     # Create a node
#     # The node "py_test" is created in this python file
#     # The python file itself is NOT a node!
#     # The node is contained in this file
#     node = Node("py_test")

#     node.get_logger().info("Hello ROS2 from Python!") 

#      # SPIN can keep the node alive
#     rclpy.spin(node)
    
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()    


# ////Object Oriented Code////////////////////////////
# Create a class that is inherited from Node
# The node class can be used later anywhere
class MyNode(Node):
    def __init__(self):
        # Call the constructor of the parent class Node
        super().__init__("py_test") # name of the node
        # The node inherits all functionality from the class
        # Now we can use self to access the node functionality

        # Create a counter
        # For attributes of a class, there is always an underscore following the name
        self.counter_ = 0
        self.get_logger().info("Hello ROS2 from Python!")
        
        # create a timer, call the callback every second while the node is spinning
        self.create_timer(1.0, self.timer_callback) 

    def timer_callback(self):
        self.get_logger().info("Hello!" + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode() # create a instance of the myNode class
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()