#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

#include <iostream>

void callback(std_msgs::String::ConstPtr msg)
{
	std::cout << "Received message: " << msg->data << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::create_node("in_process");

    auto publisher = node->create_publisher<std_msgs::String>("chatter", 0);

    auto subscription = node->create_subscription<std_msgs::String>("chatter", 10, callback);

    std_msgs::String::Ptr msg(new std_msgs::String);
    int count = 0;
    while(node->is_running())
    {
        std::stringstream ss;
        ss << "[" << count++ << "]: Hello World!";
        msg->data = ss.str();
        publisher->publish(msg);
        node->spin_once();
        sleep(1);
    }
}
