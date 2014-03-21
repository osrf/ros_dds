#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

void rclcpp::init(int argc, char** argv)
{
    return;
}

Node::Ptr rclcpp::create_node(const std::string &name)
{
    return Node::Ptr(new Node(name));
}
