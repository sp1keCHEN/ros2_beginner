#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "SUB");
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>("command", 10,  std::bind(&MySubscriber::msgCallback, this, std::placeholders::_1));
        command_service_ = this->create_service<example_interfaces::srv::AddTwoInts>("addtwoints", std::bind(&MySubscriber::serviceCallback, this,  std::placeholders::_1, std::placeholders::_2));
    }

private:

    void msgCallback(const std_msgs::msg::String::SharedPtr msgs) {
        RCLCPP_INFO(rclcpp::get_logger("Topic"), msgs->data);
    }

    void serviceCallback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
                            example_interfaces::srv::AddTwoInts::Response::SharedPtr res) {
        res->sum = req->a * req->b;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Service"), "Sum: " << res->sum);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr command_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>("subscriber");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}