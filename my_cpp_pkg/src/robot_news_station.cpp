#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp" // "s" in String is lowercase, and with surfix .hpp
                                                // Remember to modify the dependency in package.xml and CmakeLists.txt(find dependencies)
using namespace std::chrono_literals; // to use 1s, 500ms style time literals
class RobotNewsStationNode : public rclcpp::Node 
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this)); // publish every 0.5 second
        RCLCPP_INFO(this->get_logger(), "Robot News Station Node has been started.");
    }
private:
    void publishNews() // The function is private because it will only be used by a timer inside the class and depends on nothing outside the class
    {
        auto msg = example_interfaces::msg::String(); // Not a shared pointer, directly an object
        msg.data = std::string("Hello, this is ") + robot_name_ + std::string(" from the robot news station.");
        publisher_->publish(msg);
    }
    std::string robot_name_; // Remember to define the name
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
