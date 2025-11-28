#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from rclpy.parameter import Parameter

class NumberPublisherNode(Node): 
    def __init__(self):
        super().__init__("number_publisher") 
        self.declare_parameter("number", 2) # First we have to declare the parameters, "number" is the name, 2 is the default value, as well as the datatype int
        self.declare_parameter("timer_period", 1.0) # "timer_period" is the name, 1.0 is the default value, as well as the datatype float
        self.number_ = self.get_parameter("number").value # Get the value of the parameter "number", if no value input, use the default value 2
        self.timer_period_ = self.get_parameter("timer_period").value # same as above, note that it's not a method "value()", but a property "value"
        
        self.add_post_set_parameters_callback(self.parameters_callback)
        
        self.number_publisher_ = self.create_publisher(Int64, "number", 10)
        self.create_timer(self.timer_period_, self.publish_number) # Here we use the parameter's value to set the timer period
        self.get_logger().info("Number Publisher is online!")

    def publish_number(self):
        msg = Int64() 
        msg.data = self.number_ # parameter value
        self.number_publisher_.publish(msg) 
    
    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "number" and param.type_ == Parameter.Type.INTEGER:
                self.number_ = param.value
                self.get_logger().info(f"Parameter 'number' changed to: {self.number_}")
            elif param.name == "timer_period" and param.type_ == Parameter.Type.DOUBLE:
                self.timer_period_ = param.value
                self.get_logger().info(f"Parameter 'timer_period' changed to: {self.timer_period_}")
        # return rclpy.parameter.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()