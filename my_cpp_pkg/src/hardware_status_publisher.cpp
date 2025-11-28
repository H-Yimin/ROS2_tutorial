#include "rclcpp/rclcpp.hpp"
// Add the include path for the interface in /.vscode/c_cpp_properties.json
// Originally we created the interface as "HardwareStatus", in cpp it is going to create underscores for the header whenever there are upper letters
#include "my_robot_interfaces/msg/hardware_status.hpp" 

using namespace std::chrono_literals;
class HardwareStatusPublisherNode : public rclcpp::Node 
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher") 
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this)
        );
        RCLCPP_INFO(this->get_logger(), "Hardware status published.");
    }
private:
    void publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus(); 
        msg.temperature = 55.0;
        msg.are_motors_ready = false;
        msg.debug_message = "Nothing special.";
        publisher_->publish(msg);
    }


    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
