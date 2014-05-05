#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/node/node.hpp>
#include <rclcpp/publisher/publisher.hpp>
#include <boost/shared_ptr.hpp>

#include <genidlcpp/resolver.h>

#include <ccpp_dds_dcps.h>

namespace rclcpp
{
    namespace service
    {
        template <typename ROSService>
        class Service
        {
        public:
            typedef std::function<bool (typename ROSService::Request::ConstPtr, typename ROSService::Response::Ptr)> CallbackType;
            typedef std::shared_ptr< Service<ROSService> > Ptr;


            Service(const std::string& service_name, rclcpp::node::Node *node, CallbackType cb) : service_name_(service_name), node_(node), cb_(cb)
            {
                // TODO the response topic is not client specific right now!!!
                size_t queue_size = 0;
                std::string topic_name = this->service_name_ + "_response";// + req->client_id;
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
                publisher_->publish(res);
            }

        private:
            std::string service_name_;
            CallbackType cb_;
            rclcpp::node::Node *node_;
            typename publisher::Publisher<typename ROSService::Response>::Ptr publisher_;

        };
    }
}
#endif
