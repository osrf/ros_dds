#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("talker");

    rclcpp::Publisher<std_msgs::String> publisher = node.create_publisher<std_msgs::String>("chatter", 0);

    std_msgs::String msg;
    msg.data = "Hello World!";
    publisher.publish(msg);
}
