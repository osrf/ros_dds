#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

void rclcpp::init(int argc, char** argv)
{
    return;
}

void rclcpp::init(std::list<std::string> args)
{
    return;
}

Node rclcpp::create_node(std::string name)
{
    return Node(name);
}

