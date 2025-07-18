#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "SUB");
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>("command", 10,  std::bind(&MySubscriber::msgCallback, this, std::placeholders::_1));
    }

private:

    void msgCallback(std_msgs::msg::String::SharedPtr msgs) {
        RCLCPP_INFO(this->get_logger(), msgs->data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>("subscriber");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}