#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>

#include "ccpp_ROSMsg.h"

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name)
{
    this->name_ = name;
    this->dpf_ = DDS::DomainParticipantFactory::get_instance();
    DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;

    this->participant_ = this->dpf_->create_participant(
        domain, PARTICIPANT_QOS_DEFAULT, NULL,
        DDS::STATUS_MASK_NONE);
}

Node::~Node() {
    this->dpf_->delete_participant(this->participant_);
}

Publisher Node::create_publisher(std::string topic_name, size_t queue_size)
{
    ROSMessageTypeSupport_var rosMessageTS = new ROSMessageTypeSupport();
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
