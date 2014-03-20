#include <rclcpp/rclcpp.hpp>
#include <iostream>

void callback(const std::string& msg)
{
    std::cout << "Received message: " << msg << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("listener");
    rclcpp::Subscription<std::string> subscription = node.create_subscription<std::string>("rossometopic", 10, callback); 
    node.wait();
    return 0;
}
