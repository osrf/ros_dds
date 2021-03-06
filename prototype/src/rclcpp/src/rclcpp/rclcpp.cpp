#include <rclcpp/rclcpp.hpp>

#include <functional>

using namespace rclcpp;

std::list<Node *> Node::nodes_;

static bool globally_initialized = false;

void rclcpp::init(int argc, char** argv)
{
    if (globally_initialized)
    {
        throw AlreadyInitializedError();
    }
    /* Register a signal handler so DDS doesn't just sit there... */
    if (signal(SIGINT, Node::static_signal_handler) == SIG_ERR)
    {
        fputs("An error occurred while setting a signal handler.\n", stderr);
    }
    globally_initialized = true;
}

Node::Ptr rclcpp::create_node(const std::string &name)
{
    return Node::Ptr(new Node(name));
}
