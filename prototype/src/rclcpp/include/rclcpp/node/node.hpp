#ifndef RCLCPP_RCLCPP_NODE_NODE_HPP_
#define RCLCPP_RCLCPP_NODE_NODE_HPP_

#include <functional>
#include <list>
#include <map>
#include <memory>

#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <ccpp_dds_dcps.h>

#include <genidlcpp/resolver.h>

#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

#include <rclcpp/client/client.hpp>
#include <rclcpp/service/service.hpp>

#include <rclcpp/parameter/client.hpp>
#include <rclcpp/parameter/server.hpp>

#include <genidlcpp/resolver.h>

namespace rclcpp
{

// Forward declarations for friends of the Node constructor
std::shared_ptr<rclcpp::node::Node> create_node(const std::string &);
void init(int argc, char** argv);

using client::Client;
using service::Service;

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
    Node(std::string name, subscription::SubscriptionManager::Ptr);

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

    template <typename ROSMsgType>
    typename publisher::Publisher<ROSMsgType>::Ptr get_publisher(const std::string &topic_name, size_t queue_size)
    {
        typedef publisher::Publisher<ROSMsgType> Pub;
        auto publisher = this->publishers_.find(topic_name);
        if (publisher != this->publishers_.end())
        {
            return std::dynamic_pointer_cast<Pub>(publisher->second);
        }
        else
        {
            return create_publisher<ROSMsgType>(topic_name, queue_size);
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
        typedef ::dds_impl::DDSTypeResolver<ROSMsgType> r;

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
        PubIfacePtr pub(new Pub(topic_name, queue_size, dds_publisher, dds_topic, dds_topic_datawriter, this->subscription_manager_));

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
        typedef ::dds_impl::DDSTypeResolver<ROSMsgType> r;

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

        this->subscription_manager_->add(sub);

        return std::dynamic_pointer_cast<Sub>(sub);
    };

    std::list<subscription::SubscriptionInterface::Ptr> get_subscriptions() const {
        return this->subscriptions_;
    };

    /* Creates and returns a Service based on a ROS Request Type, ROS Response Type,
     * a service name.
     *
     * The service is templated on the ROS Request Type and ROS Response Type.
     *
     * Additionally, a callback is required. The callback should take this signature:
     *
     *     void callback(ROSRequestType::ConstPtr req, ROSResponseType::Ptr res);
     */
    template <typename ROSService>
    typename Service<ROSService>::Ptr create_service(const std::string &service_name, typename Service<ROSService>::CallbackType cb)
    {
        // XXX hardcoded queue_size
        size_t queue_size = 0;

        typename rclcpp::service::Service<ROSService>::Ptr service(
            new rclcpp::service::Service<ROSService>(service_name, this, cb));
        typename rclcpp::subscription::Subscription<typename ROSService::Request>::CallbackType f(
            std::bind(&rclcpp::service::Service<ROSService>::handle_request,
                      service, std::placeholders::_1));

        // Create a Subscription for the Service's request channel
        typename rclcpp::subscription::Subscription<typename ROSService::Request>::Ptr request_subscription(
            this->create_subscription<typename ROSService::Request>(service_name + "_request", queue_size, f));

        return service;
    }

    /* Creates and returns a Client based on a ROS Request Type, ROS Response Type,
     * a service name.
     *
     * The client is templated on the ROS Request Type and ROS Response Type.
     */
    template <typename ROSService>
    typename Client<ROSService>::Ptr create_client(const std::string &service_name)
    {
        // XXX hardcoded queue_size
        size_t queue_size = 0;

        boost::uuids::uuid client_id_uuid = boost::uuids::random_generator()();
        std::string client_id = boost::lexical_cast<std::string>(client_id_uuid);
        // DDS does not support topic names that contain - or #
        boost::erase_all(client_id, "-");

        std::string topic_name = service_name + "_response";// + client_id;

        typename rclcpp::publisher::Publisher<typename ROSService::Request>::Ptr publisher(
            this->create_publisher<typename ROSService::Request>(
                service_name + "_request", queue_size));

        typename rclcpp::client::Client<ROSService>::Ptr client(
            new rclcpp::client::Client<ROSService>(client_id, publisher, this));

        typename rclcpp::subscription::Subscription<typename ROSService::Response>::CallbackType f(
            std::bind(&rclcpp::client::Client<ROSService>::handle_response,
                      client, std::placeholders::_1));

        // Create a Subscription for the Client's response channel
        typename rclcpp::subscription::Subscription<typename ROSService::Response>::Ptr response_subscription(
            this->create_subscription<typename ROSService::Response>(topic_name, queue_size, f));

        return client;
    }

    /* Destroys a subscription by reference */
    template <typename ROSMsgType>
    void destroy_subscription(const typename subscription::Subscription<ROSMsgType>::Ptr subscription)
    {
        this->waitset_->detach_condition(subscription->get_status_condition());
        this->subscriptions_.remove(subscription);
        this->subscription_manager_->remove(subscription);
    }

    /* Processes subscription callbacks, blocking until shutdown (ctrl-c) */
    void spin();

    /* Process one subscription callback, if needed, and then returns */
    bool spin_once();

    /* Creates and returns a ParameterClient */
    rclcpp::parameter::ParameterClient::Ptr create_parameter_client(const std::string &prefix);

    /* Creates and returns a ParameterServer */
    rclcpp::parameter::ParameterServer::Ptr create_parameter_server(const std::string &prefix);

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

    subscription::SubscriptionManager::Ptr subscription_manager_;
};

}
}

#endif /* RCLCPP_RCLCPP_NODE_NODE_HPP_ */
