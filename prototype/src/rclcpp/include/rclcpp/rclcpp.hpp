/*      <proj>_<path>_<file>_ext_ */
#ifndef RCLCPP_RCLCPP_RCLCPP_HPP_
#define RCLCPP_RCLCPP_RCLCPP_HPP_

#include <exception>
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
node::Node::Ptr create_node(const std::string &name);

class AlreadyInitializedError : public std::exception
{
    virtual const char* what() const throw()
    {
        return "rclcpp::init called more than once";
    }
};

}

#endif /* RCLCPP_RCLCPP_RCLCPP_HPP_ */
