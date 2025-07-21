#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <thread>
#include <atomic>

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "PUB");
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);

        service_client_ = this->create_client<example_interfaces::srv::AddTwoInts>("addtwoints");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyPublisher::timerCallback, this));
        
        count_ = 0;
        running_ = true;
        client_thread_ = std::thread(&MyPublisher::clientThread, this);
    }   

    ~MyPublisher() {
        running_ = false;
        if (client_thread_.joinable()) {
            client_thread_.join();
        }
    }

private:

    void clientThread() {
        while (!service_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }

        while(running_ && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            // example_interfaces::srv::AddTwoInts::Request::SharedPtr req; // 典型错误
            auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            req->a = count_;
            req->b = count_;
            
            service_client_->async_send_request(req, std::bind(&MyPublisher::clientCallback, this, std::placeholders::_1));
            
            auto future = service_client_->async_send_request(req);
            future.wait();

            // --- timetout ---
            // std::future_status status = future.wait_for(std::chrono::seconds(5));
            // if (status == std::future_status::ready) {
            //     auto result = future.get();
            //     RCLCPP_INFO(this->get_logger(), "收到响应: %d", result->sum);
            // } else {
            //     RCLCPP_WARN(this->get_logger(), "等待服务响应超时");
            // }

            RCLCPP_INFO_STREAM(this->get_logger(), "sum" << future.get()->sum);
        }
    }

    void clientCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture res) {
        auto result = res.get();
        RCLCPP_INFO_STREAM(this->get_logger(), "sumCallback: " << result->sum);
    }

    void timerCallback() {
        count_++;
        std_msgs::msg::String pub_msgs;
        pub_msgs.data = "fxxk " + std::to_string(count_);
        command_publisher_->publish(pub_msgs);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr service_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<int> count_;
    std::atomic<bool> running_;
    std::thread client_thread_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPublisher>("publisher");

    rclcpp::spin(node);

    // --- multi callback ---
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();

    rclcpp::shutdown();
    return 0;
}
