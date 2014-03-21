#ifndef RCLCPP_RCLCPP__HPP
#define RCLCPP_RCLCPP__HPP

#include <string>

#include <rclcpp/node/node.hpp>

namespace rclcpp
{

/* Bring classes from lower namespaces up to this namespace */
using node::Node;
using publisher::Publisher;
using subscription::Subscription;

/* Global initialization function
 *
 * This function should be called once per process, before creating nodes.
 */
void init(int argc, char** argv);

/* Node factory function
 *
 * Create a new Node with a given unique name.
 */
node::Node create_node(const std::string &name);

}

#endif /* RCLCPP_RCLCPP__HPP */
