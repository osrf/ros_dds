#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::create_node("talker");

    auto publisher = node->create_publisher<std_msgs::String>("chatter", 0);

    std_msgs::String msg;
    int i = 0;
    while(i < 10)
    {
        i++;
        msg.data = "Hello World!";
        publisher->publish(msg);
        sleep(1);
    }
}
