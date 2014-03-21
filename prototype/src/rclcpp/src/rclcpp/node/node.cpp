#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>

#include <iostream>

#include <signal.h>

using namespace rclcpp::node;
using namespace rclcpp::publisher;

bool running;

static void catch_function(int signo) {
    running = false;
    std::cout << "Catching Ctrl-C, shutting down..." << std::endl;
}

Node::Node(std::string name)
{
    running = true;
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

    /* Register a signal handler so DDS doesn't just sit there... */
    if (signal(SIGINT, catch_function) == SIG_ERR)
    {
        fputs("An error occurred while setting a signal handler.\n", stderr);
    }
}

void Node::spin()
{
    while(running)
    {
        this->spin_once();
    }
}

void Node::spin_once()
{
    for (auto iterator = this->subscriptions_.cbegin(); iterator != this->subscriptions_.cend(); ++iterator)
    {
        (*iterator)->spin_once();
    }
}

Node::~Node() {
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
