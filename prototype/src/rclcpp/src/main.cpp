#include <rclcpp/rclcpp.hpp>
#include <iostream>

void callback(const std::string& msg)
{
    std::cout << "Received message: " << msg << std::endl;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("talker");
    rclcpp::Publisher publisher = node.create_publisher("rossometopic", 10);
    rclcpp::Subscription<std::string> subscription = node.create_subscription<std::string>("rossometopic", 10, callback); 
    while(true)
    for(int i=0;i < 10000; ++i)
    {
        publisher.publish<std::string>("Hello world " + std::string(argv[1]));
        usleep(100000);
    }
    node.wait();
    return 0;
}
