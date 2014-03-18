#include <rclcpp/node/node.hpp>

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name)
{
    this->name_ = name;
}

Node::~Node() {}

Publisher Node::create_publisher(std::string topic_name, size_t queue_size)
{
    if (this->publishers_.find(topic_name) != this->publishers_.end())
    {
        // Raise, already called for topic
    }
    this->publishers_.insert(std::pair<std::string, Publisher>(topic_name, Publisher(topic_name, queue_size)));
    return this->publishers_.at(topic_name);
}

void Node::destroy_publisher(Publisher publisher)
{
    this->destroy_publisher(publisher.get_topic_name());
}

void Node::destroy_publisher(std::string topic_name)
{
    if (this->publishers_.find(topic_name) == this->publishers_.end())
    {
        // Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}
