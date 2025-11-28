#include "rclcpp/rclcpp.hpp"

// ////Minimal Code////
// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     // Create a shared pointer to a Node object
//     auto node = std::make_shared<rclcpp::Node>("cpp_test");

//     // To print the log, use RCLCPP_INFO
//     // Use arrow to use the node functionality
//     // if use node.xxx it will use the shared pointer functionality
//     RCLCPP_INFO(node->get_logger(), "Hello ROS from CPP!");
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

////OOP///
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test")
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS from CPP!"); 
        // "this" is a pointer to the current node cpp_test, optional
        // the node inherits the function from MyNode class
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this)  
                                         // bind the member function to the current object
        );
    }
private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
        counter_ ++;
    }

    // Create a timer to call the timerCallback function every second
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>(); //No need to pass the node name here
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
