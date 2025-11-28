#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # There are also AddTwoInts_Request and AddTwoInts_Response

class AddTwoIntsNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_server") 
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        # It's recommended to use a verb for the service name, e.g., "add", "compute", "calculate", etc.
        # A callback function is called when a service request is received
        self.get_logger().info("Service 'add_two_ints' has been started.")
    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response): # Make the parameter type more explicit
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response # Return the response object. If forgotten, we will get exception from the callback

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()