#ifndef SERVICE_HPP
#define SERVICE_HPP

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
            typedef boost::shared_ptr<const ROSRequestType> req_shared_ptr;
            typedef boost::shared_ptr<const ROSResponseType> res_shared_ptr;
            typedef boost::function<bool (const ROSRequestType&, ROSResponseType&)> CallbackType;
            typedef boost::shared_ptr< Service<ROSRequestType, ROSResponseType> > shared_service;


            Service(const std::string& service_name, rclcpp::Node *node, CallbackType cb, typename Publisher<ROSResponseType>::shared_publisher publisher) : service_name_(service_name), node_(node), cb_(cb), publisher_(publisher) {}

            ~Service() {}

           void handle_request(const ROSRequestType& req)
           {
               ROSResponseType res;
               this->cb_(req, res);
               res.req_id = req.req_id;
               res.client_id = req.client_id;
               publisher_->publish(res);
           }

        private:
            std::string service_name_;
            CallbackType cb_;
            rclcpp::Node *node_;

            typename Publisher<ROSResponseType>::shared_publisher publisher_;
        };
    }
}
#endif
