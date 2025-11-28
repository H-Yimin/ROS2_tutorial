#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool # import the interface

class NumberCounterNode(Node): 
    def __init__(self):
        super().__init__("number_counter")
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10) 
        self.counter_ = 0
        self.number_count_publisher_ = self.create_publisher(
            Int64, "number_count", 10)
        self.reset_counter_service_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter)   # Create a server
        self.get_logger().info("Number Counter is online!")

    def callback_number(self, msg: Int64):
        self.counter_ += msg.data
        # self.get_logger().info(f"Received number: {self.counter_}")
        new_msg = Int64()
        new_msg.data = self.counter_
        self.number_count_publisher_.publish(new_msg)
    
    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response): # make it explicit
        if request.data: # If the request is True
            self.counter_ = 0
            response.success = True
            response.message = "Counter has been reset."

        else: # If the request is False
            response.success = False # Here we set success to False for simplicity, in real case we may want to e.g. enable a motor,
                                    # We need to do some checks before reset, and if checks fail, we could return a response with a message describing the issue
            response.message = "reset has not been reset."
        return response # !!!

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()