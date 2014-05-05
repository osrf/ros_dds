#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::create_node("parameter_server");
    node->spin();
    return 0;
}
