#include <list>
#include <string>

#include <rclcpp/node/node.hpp>

namespace rclcpp
{
    typedef node::Node Node;
    using publisher::Publisher;
    // using subscription::Subscription;

    void init(int argc, char** argv);
    void init(std::list<std::string> args);

    Node create_node(std::string name);
}
