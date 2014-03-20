#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>

#include "ccpp_ROSMsg.h"

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name)
{
    // XXX move this code to create_node?
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
    // TODO check that topic_name is valid (i.e. no slashes)

    // TODO check return status
    DDS::ReturnCode_t status;

    // TODO define partiion name in common location
    std::string partition_name = "ros_partition";

    DDS::TopicQos default_topic_qos;
    status = this->participant_->get_default_topic_qos(default_topic_qos);
    default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;

    ROSMessageTypeSupport_var ros_message_ts = new ROSMessageTypeSupport();
    char * ros_message_type_name = ros_message_ts->get_type_name();
    status = ros_message_ts->register_type(
        this->participant_.in(), ros_message_type_name);

    DDS::PublisherQos publisher_qos;
    status = this->participant_->get_default_publisher_qos(publisher_qos);
    publisher_qos.partition.name.length(1);
    publisher_qos.partition.name[0] = partition_name.c_str();

    DDS::Publisher_var dds_publisher = this->participant_->create_publisher(
        publisher_qos, NULL, DDS::STATUS_MASK_NONE);

    DDS::Topic_var ros_message_topic = this->participant_->create_topic(
        topic_name.c_str(), ros_message_type_name, default_topic_qos, NULL,
        DDS::STATUS_MASK_NONE
    );

    DDS::DataWriter_var topic_writer = dds_publisher->create_datawriter(
        ros_message_topic.in(), DATAWRITER_QOS_USE_TOPIC_QOS,
        NULL, DDS::STATUS_MASK_NONE);

    ROSMessageDataWriter_var data_writer = ROSMessageDataWriter::_narrow(topic_writer.in());

    if (this->publishers_.find(topic_name) != this->publishers_.end())
    {
        // TODO Raise, already called for topic
    }
    Publisher publisher(topic_name, queue_size, dds_publisher, ros_message_topic,
        topic_writer, data_writer);

    this->publishers_.insert(std::pair<std::string, Publisher>(topic_name, publisher));
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
        // TODO Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}
