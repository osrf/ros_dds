#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/impl/String_pubsub.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::create_node("talker");

    rclcpp::Publisher::Ptr publisher = node->create_publisher<std_msgs::String>("chatter", 0);

    int count = 0;
    while(node->is_running())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "[" << count++ << "]: Hello World!";
        msg.data = ss.str();

        std::cout << "Sending: " << msg.data << std::endl;

        publisher->publish(msg);

        sleep(1);
    }
}
