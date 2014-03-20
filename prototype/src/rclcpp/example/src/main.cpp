#include <rclcpp/rclcpp.hpp>

#include <std_msgs/Int32.h>
#include "std_msgs/dds_impl/Int32_convert.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("example");

    rclcpp::Publisher<std_msgs::Int32> publisher = node.create_publisher<std_msgs::Int32>("temp", 0);

    std_msgs::Int32 msg;
    msg.data = 2;
    publisher.publish(msg);

}
