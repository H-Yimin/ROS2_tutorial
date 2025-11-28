#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")
    
    client = node.create_client(AddTwoInts, "add_two_ints") # make sure use the same service name
    # Wait until the service is up
    while not client.wait_for_service(1.0): # 1.0 second timeout
        node.get_logger().warn("Waiting for Add Two Ints server...")

    # Create a request object
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    # Send the request and get the response 
    future = client.call_async(request) # Non-blocking call, if we use client.call(request), it will block until get the response, because it's a synchronous call
                                        # future object is like the response from the server
    rclpy.spin_until_future_complete(node, future) # Spin until the future is complete
    response = future.result()
    node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))


    rclpy.shutdown()

if __name__ == "__main__":
    main()