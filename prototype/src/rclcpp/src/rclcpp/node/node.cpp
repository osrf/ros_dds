#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>

#include <iostream>

#include <signal.h>

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name)
{
    this->nodes_.push_back(this);
    this->subscription_iterator_ = this->subscriptions_.end();
    this->name_ = name;
    this->dpf_ = DDS::DomainParticipantFactory::get_instance();
    checkHandle(this->dpf_.in(), "DDS::DomainParticipantFactory::get_instance");
    DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;

    this->participant_ = this->dpf_->create_participant(
        domain, PARTICIPANT_QOS_DEFAULT, NULL,
        DDS::STATUS_MASK_NONE);
    checkHandle(this->participant_.in(), "DDS::DomainParticipantFactory::create_participant");

    // Create the default QoS for Topics
    DDS::ReturnCode_t status = this->participant_->get_default_topic_qos(this->default_topic_qos_);
    checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
    this->default_topic_qos_.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

    // Create the default QoS for Publishers
    status = this->participant_->get_default_publisher_qos(this->default_publisher_qos_);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    this->default_publisher_qos_.partition.name.length(1);
    this->default_publisher_qos_.partition.name[0] = "ros_partition";

    // Create the default QoS for Subscribers
    status = this->participant_->get_default_subscriber_qos(this->default_subscriber_qos_);
    checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    this->default_subscriber_qos_.partition.name.length(1);
    this->default_subscriber_qos_.partition.name[0] = "ros_partition";
}

void Node::spin()
{
    while(this->running_)
    {
        this->spin_once();
    }
}

bool Node::spin_once()
{
    if (this->subscription_iterator_ == this->subscriptions_.end())
    {
        this->subscription_iterator_ = this->subscriptions_.begin();
    }
    bool did_spin;
    if (this->subscription_iterator_ != this->subscriptions_.end())
    {
        did_spin = (*this->subscription_iterator_)->spin_once();
        auto current_it = this->subscription_iterator_;
        // Increament for the next loop to start on someone else
        ++this->subscription_iterator_;
        // If it did spin, return true
        if (did_spin)
        {
            return true;
        }
        // If it did not spin
        // Loop from the start to the end looking for one to spin
        for (auto it = current_it; it != this->subscriptions_.end(); ++it)
        {
            did_spin = (*it)->spin_once();
            if (did_spin)
            {
                return true;
            }
        }
        // If it still did not spin, loop from beginning to start
        for (auto it = this->subscriptions_.begin(); it != current_it; ++it)
        {
            did_spin = (*it)->spin_once();
            if (did_spin)
            {
                return true;
            }
        }
    }
    return false;
}

void Node::static_signal_handler(int signo)
{
    std::cout << "Catching SIGINT (ctrl-c), shutting down nodes..." << std::endl;
    for (auto it = Node::nodes_.begin(); it != Node::nodes_.end(); ++it)
    {
        (*it)->shutdown("Caught SIGINT (ctrl-c)");
    }
}

void Node::shutdown(const std::string &reason)
{
    this->running_ = false;
    this->shutdown_reason_ = reason;
}

Node::~Node()
{
    this->dpf_->delete_participant(this->participant_);
}

void Node::destroy_publisher(const rclcpp::publisher::PublisherInterface::Ptr &publisher_interface)
{
    this->destroy_publisher(publisher_interface->get_topic_name());
}

void Node::destroy_publisher(std::string topic_name)
{
    if (this->publishers_.find(topic_name) == this->publishers_.end())
    {
        // TODO Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}
