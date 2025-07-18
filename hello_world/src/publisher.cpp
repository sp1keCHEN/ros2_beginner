#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "PUB");
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyPublisher::timerCallback, this));
    }

private:

    void timerCallback() {
        std_msgs::msg::String pub_msgs;
        pub_msgs.data = "fxxk" + std::to_string(count);
        command_publisher_->publish(pub_msgs);
        count++;
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPublisher>("publisher");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
