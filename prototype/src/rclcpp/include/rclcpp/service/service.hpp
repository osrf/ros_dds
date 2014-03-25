#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

#include <rclcpp/publisher/publisher.hpp>
#include <boost/shared_ptr.hpp>

#include <genidlcpp/resolver.h>

#include <ccpp_dds_dcps.h>

namespace rclcpp
{
    namespace service
    {
        template <typename ROSRequestType, typename ROSResponseType>
        class Service
        {
        public:
            typedef std::shared_ptr<const ROSRequestType> req_shared_ptr;
            typedef std::shared_ptr<const ROSResponseType> res_shared_ptr;
            typedef std::function<bool (const ROSRequestType&, ROSResponseType&)> CallbackType;
            typedef std::shared_ptr< Service<ROSRequestType, ROSResponseType> > Ptr;


            Service(const std::string& service_name, rclcpp::node::Node *node, CallbackType cb, typename rclcpp::publisher::Publisher<ROSResponseType>::Ptr publisher) : service_name_(service_name), node_(node), cb_(cb), publisher_(publisher) {}

            ~Service() {}

            void handle_request(typename ROSRequestType::ConstPtr req)
            {
/*
                ROSResponseType res;
                this->cb_(req, res);
                res.req_id = req->req_id;
                res.client_id = req->client_id;
                this->publisher_->publish(res);
*/
            }

        private:
            std::string service_name_;
            CallbackType cb_;
            rclcpp::node::Node *node_;

            typename rclcpp::publisher::Publisher<ROSResponseType>::Ptr publisher_;
        };
    }
}
#endif
