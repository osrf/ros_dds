#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("talker");
    rclcpp::Publisher publisher = node.create_publisher("rossometopic", 10);
    while(true)
    for(int i=0;i < 10000; ++i)
    {
        publisher.publish<std::string>("Hello world " + std::string(argv[1]));
        usleep(100000);
    }
    return 0;
}
