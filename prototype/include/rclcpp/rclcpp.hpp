#include <list>
#include <string>

#include <rclcpp/node/node.hpp>

namespace rclcpp
{
    void init(int argc, char* argv[]);
    void init(std::list<std::string> args);

    node::Node create_node(std::string name);
}
