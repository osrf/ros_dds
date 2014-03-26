#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/impl/String_pubsub.hpp"

#include <iostream>

void callback(std_msgs::String::ConstPtr msg)
{
	std::cout << "Received message: " << msg->data << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::create_node("listener");

    auto subscription = node->create_subscription<std_msgs::String>("chatter", 10, callback);

    std::cout << "Listener Ready, spinning..." << std::endl;

    node->spin();

    return 0;
}
