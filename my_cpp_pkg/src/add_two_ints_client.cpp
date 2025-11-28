#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
class AddTwoIntsClientNode : public rclcpp::Node 
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client") 
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void callAddTwoInts(int a, int b)
    {
        while (!client_->wait_for_service(1s)) 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to be available...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        
        // last_a_ = a;  // store the values to use in the callback
        // last_b_ = b;
        
        request->a = a; // parameters from the function
        request->b = b;

        client_->async_send_request(
            request, std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts, this, _1));
    }
private:
    void callbackCallAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "%d", (int)response->sum);
        // RCLCPP_INFO(this->get_logger(), "%d + %d = %d",
        //             last_a_, last_b_, (int)response->sum);
        // There is a problem here if we make multiple calls before the previous ones return,
        // the last_a_ and last_b_ will be overwritten.
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    // int last_a_;
    // int last_b_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>(); 
    node->callAddTwoInts(10, 5);
    node->callAddTwoInts(12, 7);
    node->callAddTwoInts(1, 2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
