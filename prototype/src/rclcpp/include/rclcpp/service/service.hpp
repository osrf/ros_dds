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
            typedef boost::function<bool (ROSRequestType, ROSResponseType)> CallbackType;

            Service(const std::string& service_name, rclcpp::Node *node, CallbackType cb) : service_name_(service_name), node_(node), cb_(cb) {}

            ~Service() {}

           void handle_request(const ROSRequestType& request)
           {
               ROSResponseType response;
               this->cb_(request, response);
           }

        private:
            std::string service_name_;
            CallbackType cb_;
            rclcpp::Node *node_;
        };
    }
}
#endif
