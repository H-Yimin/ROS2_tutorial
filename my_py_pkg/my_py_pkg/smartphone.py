#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Smartphone(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone node is online!")

    # Create a function/method so that when a msg is received, it is called
    # In the following we will print the received msg
    # In real cases the node may monitor the temperature of several sensors,
    # when the temperature exceeds a threshold, it will take some action, e.g. trigger an alarm
    def callback_robot_news(self, msg: String):
        self.get_logger().info(msg.data)
        # in the String object, we have build "data" in the publisher file

def main(args=None):
    rclpy.init(args=args)
    node = Smartphone() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()