#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

#include <iostream>

void callback(const std_msgs::String &msg)
{
    std::cout << "Received message: " << msg << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("listener");
    boost::shared_ptr< rclcpp::Subscription<std_msgs::String> > subscription(node.create_subscription<std_msgs::String>("chatter", 10, callback));
    node.wait();
    return 0;
}
