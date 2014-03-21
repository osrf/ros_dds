#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

void rclcpp::init(int argc, char** argv)
{
    return;
}

rclcpp::node::Node rclcpp::create_node(const std::string &name)
{
    return Node(name);
}
