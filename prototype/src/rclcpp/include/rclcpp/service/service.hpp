#ifndef RCLCPP_RCLCPP_SERVICE_SERVICE_HPP_
#define RCLCPP_RCLCPP_SERVICE_SERVICE_HPP_

#include <functional>
#include <memory>

#include <rclcpp/publisher/publisher.hpp>

#include <boost/shared_ptr.hpp>

namespace rclcpp
{

namespace node {class Node;}

namespace service
{

template <typename ROSService>
class Service
{
public:
    typedef std::function<bool(typename ROSService::Request::ConstPtr, typename ROSService::Response::Ptr)> CallbackType;
    typedef std::shared_ptr< Service<ROSService> > Ptr;

    Service(
        const std::string& service_name,
        rclcpp::node::Node *node,
        CallbackType cb)
    : service_name_(service_name), node_(node), cb_(cb)
    {
        // TODO the response topic is not client specific right now!!!
        size_t queue_size = 0;
        std::string topic_name = this->service_name_ + "_response";// + req->client_id;
        std::cout << "Publishing server responses to topic named: " << topic_name << std::endl;
        publisher_ = this->node_->template get_publisher<typename ROSService::Response>(
            topic_name, queue_size);
    }

    ~Service() {}

    void handle_request(typename ROSService::Request::ConstPtr req)
    {
        typename ROSService::Response::Ptr res(new typename ROSService::Response());
        this->cb_(req, res);
        res->req_id = req->req_id;
        res->client_id = req->client_id;
        publisher_->publish((*res));
    }

private:
    std::string service_name_;
    CallbackType cb_;
    rclcpp::node::Node *node_;
    publisher::Publisher::Ptr publisher_;

};

} // namespace service
} // namespace rclcpp

#endif /* RCLCPP_RCLCPP_SERVICE_SERVICE_HPP_ */
