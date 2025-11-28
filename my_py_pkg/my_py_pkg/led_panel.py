#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [0,0,0])
        self.led_states_ = self.get_parameter("led_states").value
        self.led_states_pub_ = self.create_publisher(
            LedStateArray, "led_panel_state", 10)
        self.led_states_timer_ = self.create_timer(
            5.0, self.publish_led_states) # callback for the timer every 5 seconds
        self.led_states_service_ = self.create_service(
            SetLed, "set_led", self.callback_set_led) # callback for the service whenever a request from the client is received
        self.get_logger().info("LED panel state node has been started.")
    
    def publish_led_states(self):
        msg = LedStateArray()
        msg.led_states = self.led_states_
        self.led_states_pub_.publish(msg)

    def callback_set_led(self, request: SetLed.Request, reponse: SetLed.Response):
        led_number = request.led_number
        state = request.state

        if led_number >= len(self.led_states_) or led_number < 0: # If the led number is out of range, other than 0,1,2
            reponse.success = False
            return reponse
        if state not in [0, 1]:
            reponse.success = False
            return reponse
        
        self.led_states_[led_number] = state
        reponse.success = True
        self.publish_led_states() # Publish the updated LED states immediately, instead of waiting for extra 5s
        return reponse

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()