#ifndef RCLCPP_RCLCPP_NODE_NODE_HPP_
#define RCLCPP_RCLCPP_NODE_NODE_HPP_

#include <list>
#include <map>
#include <memory>
#include <functional>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

#include <rclcpp/client/client.hpp>
#include <rclcpp/service/service.hpp>

namespace rclcpp
{

namespace node {class Node;}

/* Forward declarations for friends of the Node constructor */
std::shared_ptr<node::Node> create_node(const std::string &);
void init(int argc, char** argv);

using client::Client;
using service::Service;

namespace node
{

/* Forward declaration of imlpementation Node class */
namespace impl {class NodeImpl;}

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
public:
    typedef std::shared_ptr<Node> Ptr;
private:
    friend Node::Ptr rclcpp::create_node(const std::string &name);
    Node(const std::string &name);

    Node(const Node &) = delete;
public:
    ~Node();

    void shutdown(const std::string &reason="No reason given");

    bool is_running();

    std::string get_shutdown_reason();

    template <typename ROSMsgType>
    publisher::Publisher::Ptr get_publisher(const std::string &topic_name, size_t queue_size)
    {
        typedef publisher::Publisher Pub;
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
    publisher::Publisher::Ptr create_publisher(const std::string &topic_name, size_t queue_size);

    /* Destroys a publisher by reference */
    void destroy_publisher(const rclcpp::publisher::Publisher::Ptr publisher);
    /* Destroys a publisher by topic name */
    void destroy_publisher(const std::string &topic_name);

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
    subscription::Subscription::Ptr create_subscription(
        const std::string &topic_name,
        size_t queue_size,
        std::function<void(typename ROSMsgType::ConstPtr)> callback
    );

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
    typename Service<ROSService>::Ptr create_service(
        const std::string &service_name,
        std::function<void(typename ROSService::ConstPtr)> cb)
    {
        // XXX hardcoded queue_size
        size_t queue_size = 0;

        typename rclcpp::service::Service<ROSService>::Ptr service(
            new rclcpp::service::Service<ROSService>(service_name, this, cb));
        std::function<void(typename ROSService::ConstPtr)> f(
            std::bind(&rclcpp::service::Service<ROSService>::handle_request,
                      service, std::placeholders::_1));

        // Create a Subscription for the Service's request channel
        rclcpp::subscription::Subscription::Ptr request_subscription(
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
        std::cout << "Subscribed for responses to topic named: " << topic_name << std::endl;

        rclcpp::publisher::Publisher::Ptr publisher(
            this->create_publisher<typename ROSService::Request>(
                service_name + "_request", queue_size));

        typename rclcpp::client::Client<ROSService>::Ptr client(
            new rclcpp::client::Client<ROSService>(client_id, publisher, this));

        std::function<void(typename ROSService::ConstPtr)> f(
            std::bind(&rclcpp::client::Client<ROSService>::handle_response,
                      client, std::placeholders::_1));

        // Create a Subscription for the Client's response channel
        rclcpp::subscription::Subscription::Ptr response_subscription(
            this->create_subscription<typename ROSService::Response>(topic_name, queue_size, f));

        return client;
    }

    /* Destroys a subscription by reference */
    void destroy_subscription(const rclcpp::subscription::Subscription::Ptr subscription);

    /* Processes subscription callbacks, blocking until shutdown (ctrl-c) */
    void spin();

    /* Process one subscription callback, if needed, and then returns */
    bool spin_once();
private:
    friend class impl::NodeImpl;
    friend void rclcpp::init(int argc, char** argv);
    static void static_signal_handler(int signo);
    static std::list<Node *> nodes_;

    std::string name_;
    bool running_;
    std::string shutdown_reason_;

    std::map<std::string, publisher::Publisher::Ptr > publishers_;
    std::list<subscription::Subscription::Ptr> subscriptions_;
    std::list<subscription::Subscription::Ptr>::const_iterator subscription_iterator_;

    impl::NodeImpl * impl_;
};

}
}

#endif /* RCLCPP_RCLCPP_NODE_NODE_HPP_ */
