#ifndef RCLCPP_RCLCPP_NODE_NODE_HPP_
#define RCLCPP_RCLCPP_NODE_NODE_HPP_

#include <list>
#include <map>
#include <memory>

#include <ccpp_dds_dcps.h>

#include <genidlcpp/resolver.h>

#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

namespace rclcpp
{

namespace node
{

/* This class represents a single, addressable node in the ROS computation graph
 *
 * The Node class is the single point of entry into the ROS graph.
 * Upon creation, a Node is visible in the ROS Graph, and can be introspected
 * by others in the ROS graph.
 * The Node class is the single point of entry for things like creating
 * publishers, making subscriptions, providing services, using services, etc.
 */
class Node
{
private:
    friend Node create_node(const std::string &name);
public:
    Node(std::string name);
    ~Node();

    template <typename ROSMsgType>
    publisher::Publisher<ROSMsgType> create_publisher(std::string topic_name, size_t queue_size)
    {
        typedef dds_impl::DDSTypeResolver<ROSMsgType> r;

        typename r::DDSMsgTypeSupportType dds_msg_ts;
        char * dds_msg_name = dds_msg_ts.get_type_name();
        DDS::ReturnCode_t status = dds_msg_ts.register_type(this->participant_.in(), dds_msg_name);
        checkStatus(status, "TypeSupport::register_type");

        DDS::Publisher_var dds_publisher = this->participant_->create_publisher(
            this->default_publisher_qos_, NULL, DDS::STATUS_MASK_NONE);
        checkHandle(dds_publisher.in(), "DDS::DomainParticipant::create_publisher");

        DDS::Topic_var dds_topic = this->participant_->create_topic(
            topic_name.c_str(), dds_msg_name, this->default_topic_qos_, NULL,
            DDS::STATUS_MASK_NONE
        );
        checkHandle(dds_topic.in(), "DDS::DomainParticipant::create_topic");

        DDS::DataWriter_var dds_topic_datawriter = dds_publisher->create_datawriter(
            dds_topic.in(), DATAWRITER_QOS_USE_TOPIC_QOS,
            NULL, DDS::STATUS_MASK_NONE);
        checkHandle(dds_topic_datawriter.in(), "DDS::Publisher::create_datawriter");

        if (this->publishers_.find(topic_name) != this->publishers_.end())
        {
            throw publisher::DuplicatePublisherException();
        }
        return publisher::Publisher<ROSMsgType>(topic_name, queue_size, dds_publisher, dds_topic, dds_topic_datawriter);
    }

    void destroy_publisher(publisher::PublisherInterface * publisher);
    void destroy_publisher(std::string topic_name);

    template <typename ROSMsgType>
    subscription::Subscription<ROSMsgType> create_subscription(
        std::string topic_name,
        size_t queue_size,
        typename subscription::Subscription<ROSMsgType>::CallbackType cb
    )
    {
        typedef dds_impl::DDSTypeResolver<ROSMsgType> r;

        typename r::DDSMsgTypeSupportType dds_msg_ts;
        char * dds_msg_name = dds_msg_ts.get_type_name();
        DDS::ReturnCode_t status = dds_msg_ts.register_type(this->participant_.in(), dds_msg_name);

        DDS::Subscriber_var dds_subscriber = this->participant_->create_subscriber(
            this->default_subscriber_qos_, NULL, DDS::STATUS_MASK_NONE);

        DDS::Topic_var dds_topic = this->participant_->create_topic(
            topic_name.c_str(), dds_msg_name, this->default_topic_qos_, NULL,
            DDS::STATUS_MASK_NONE
        );

        DDS::DataReader_var topic_reader = dds_subscriber->create_datareader(
            dds_topic.in(), DATAREADER_QOS_USE_TOPIC_QOS,
            NULL, DDS::STATUS_MASK_NONE);

        typename r::DDSMsgDataReaderType_var data_reader = r::DDSMsgDataReaderType::_narrow(topic_reader.in());

        subscription::Subscription<ROSMsgType> subscription(data_reader, cb);
        subscription::SubscriptionInterface *subscription_if = &subscription;
        this->subscriptions_.push_back(subscription_if);
        return subscription;
    };

    template <typename ROSMsgType>
    void destroy_subscription(subscription::Subscription<ROSMsgType> subscription);

    void spin();
    void spin_once();
private:
    std::string name_;
    DDS::DomainParticipantFactory_var dpf_;
    DDS::DomainParticipant_var participant_;
    DDS::TopicQos default_topic_qos_;
    DDS::PublisherQos default_publisher_qos_;
    DDS::SubscriberQos default_subscriber_qos_;

    std::map<std::string, publisher::PublisherInterface* > publishers_;
    std::list<subscription::SubscriptionInterface *> subscriptions_;

    void subscription_watcher();
};

}
}

#endif /* RCLCPP_RCLCPP_NODE_NODE_HPP_ */
