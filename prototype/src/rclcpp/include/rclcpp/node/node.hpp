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

// Forward declarations for friends of the Node constructor
std::shared_ptr<rclcpp::node::Node> create_node(const std::string &);
void init(int argc, char** argv);

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
    friend std::shared_ptr<rclcpp::node::Node> rclcpp::create_node(const std::string &name);
    Node(std::string name);

    Node(const Node &) = delete;
public:
    typedef std::shared_ptr<Node> Ptr;
    ~Node();

    void shutdown(const std::string &reason="No reason given");

    bool is_running()
    {
        return this->running_;
    }

    std::string get_shutdown_reason()
    {
        if (this->running_)
        {
            return "";
        }
        else
        {
            return this->shutdown_reason_;
        }
    }

    /* Creates and returns a Publisher based on a ROS Msg Type and a topic name
     *
     * The Publisher is templated on the ROS Msg Type, and there can only be one
     * Publisher per topic. Therefore, topics are inherently strongly typed to
     * exactly one ROS Msg Type.
     *
     * In addition to the topic_name and the ROSMsgType template argument, a
     * queue_size must be provided.
     */
    template <typename ROSMsgType>
    typename publisher::Publisher<ROSMsgType>::Ptr create_publisher(const std::string &topic_name, size_t queue_size)
    {
        /* Ensure that a publisher for this topic does not already exist */
        if (this->publishers_.find(topic_name) != this->publishers_.end())
        {
            throw publisher::DuplicatePublisherException();
        }

        /* Deduce DDS types for the given ROSMsgType using the DDSTypeResolver */
        typedef dds_impl::DDSTypeResolver<ROSMsgType> r;

        /* Create a TypeSupport object for the equivalent DDS Msg Type */
        typename r::DDSMsgTypeSupportType dds_msg_ts;
        char * dds_msg_name = dds_msg_ts.get_type_name();
        DDS::ReturnCode_t status = dds_msg_ts.register_type(this->participant_.in(), dds_msg_name);
        checkStatus(status, "TypeSupport::register_type");

        /* Create the DDS publisher obj */
        DDS::Publisher_var dds_publisher = this->participant_->create_publisher(
            this->default_publisher_qos_, NULL, DDS::STATUS_MASK_NONE);
        checkHandle(dds_publisher.in(), "DDS::DomainParticipant::create_publisher");

        /* Create the DDS topic obj */
        DDS::Topic_var dds_topic = this->participant_->create_topic(
            topic_name.c_str(), dds_msg_name, this->default_topic_qos_, NULL,
            DDS::STATUS_MASK_NONE
        );
        checkHandle(dds_topic.in(), "DDS::DomainParticipant::create_topic");

        /* Create the DDS DataWriter for the DDS Msg Type */
        DDS::DataWriter_var dds_topic_datawriter = dds_publisher->create_datawriter(
            dds_topic.in(), DATAWRITER_QOS_USE_TOPIC_QOS,
            NULL, DDS::STATUS_MASK_NONE);
        checkHandle(dds_topic_datawriter.in(), "DDS::Publisher::create_datawriter");

        typedef publisher::Publisher<ROSMsgType> Pub;
        typedef publisher::PublisherInterface::Ptr PubIfacePtr;
        PubIfacePtr pub(new Pub(topic_name, queue_size, dds_publisher, dds_topic, dds_topic_datawriter));

        this->publishers_.insert(std::pair<std::string, PubIfacePtr>(topic_name, pub));

        return std::dynamic_pointer_cast<Pub>(pub);
    }

    /* Destroys a publisher by reference */
    template <typename ROSMsgType>
    void destroy_publisher(const typename publisher::Publisher<ROSMsgType>::Ptr &publisher)
    {
        this->destroy_publisher(publisher->get_topic_name());
    }
    /* Destroys a publisher by pointer to the base class */
    void destroy_publisher(const publisher::PublisherInterface::Ptr &publisher_interface);
    /* Destroys a publisher by topic name */
    void destroy_publisher(std::string topic_name);

    /* Creates and returns a Subscription based on a ROS Msg Type and topic
     *
     * The subscription is templated on the ROS Msg Type.
     *
     * Additionally, a queue_size and callback are required. The callback
     * should take this signature:
     *
     *     void callback(const ROSMsgType &msg);
     */
    template <typename ROSMsgType>
    typename subscription::Subscription<ROSMsgType>::Ptr create_subscription(
        std::string topic_name,
        size_t queue_size,
        typename subscription::Subscription<ROSMsgType>::CallbackType callback
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

        typedef subscription::Subscription<ROSMsgType> Sub;
        typedef subscription::SubscriptionInterface SubIface;
        typename SubIface::Ptr sub(new Sub(topic_name, data_reader, callback));
        this->subscriptions_.push_back(sub);
        // Reset the iterator on the subscriptions
        this->subscription_iterator_ = this->subscriptions_.begin();
        // Hook up the read condition to the node's waitset
        this->waitset_->attach_condition(sub->get_status_condition());
        return std::dynamic_pointer_cast<Sub>(sub);
    };

    template <typename ROSRequestType, typename ROSResponseType>
    Service<ROSRequestType, ROSResponseType> create_service(const std::string &service_name, typename Service::CallbackType<ROSRequestType, ROSResponseType> cb);

    template <typename ROSRequestType, typename ROSResponseType>
    Client<ROSRequestType, ROSResponseType> create_client(const std::string &service_name);

    /* Destroys a subscription by reference */
    template <typename ROSMsgType>
    void destroy_subscription(const typename subscription::Subscription<ROSMsgType>::Ptr subscription)
    {
        this->waitset_->detach_condition(subscription->get_status_condition());
        this->subscriptions_.remove(subscription);
    }

    /* Processes subscription callbacks, blocking until shutdown (ctrl-c) */
    void spin();

    /* Process one subscription callback, if needed, and then returns */
    bool spin_once();
private:
    std::string name_;
    DDS::DomainParticipantFactory_var dpf_;
    DDS::DomainParticipant_var participant_;
    DDS::TopicQos default_topic_qos_;
    DDS::PublisherQos default_publisher_qos_;
    DDS::SubscriberQos default_subscriber_qos_;

    std::map<std::string, publisher::PublisherInterface::Ptr > publishers_;
    std::list<subscription::SubscriptionInterface::Ptr> subscriptions_;
    std::list<subscription::SubscriptionInterface::Ptr>::const_iterator subscription_iterator_;

    static std::list<Node *> nodes_;

    friend void rclcpp::init(int argc, char** argv);
    static void static_signal_handler(int signo);

    bool running_;
    std::string shutdown_reason_;

    DDS::WaitSet * waitset_;
};

}
}

#endif /* RCLCPP_RCLCPP_NODE_NODE_HPP_ */
