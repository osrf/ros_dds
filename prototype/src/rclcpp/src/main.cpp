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
    publisher.publish<std::string>("Hello world");
    rclcpp::Subscription<std::string> subscription = node.create_subscription<std::string>("rossometopic", 10, callback); 
    rclcpp::subscription::SubscriptionInterface *subscription_interface = (&subscription);
    for(int i=0;i < 10000; ++i)
    {
        subscription_interface->spin();
        usleep(100000);
    }

    return 0;
}
