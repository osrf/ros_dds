#ifndef PARAMETER_SERVER_HPP
#define PARAMETER_SERVER_HPP

#include <rclcpp/service/service.hpp>
#include <cstdint>

#include <boost/any.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/ParameterServerGetString.h>
#include <rclcpp/ParameterServerSetString.h>

#include "rclcpp/dds_impl/ParameterServerGetStringRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetStringResponse_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetStringRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetStringResponse_convert.h"

#include <rclcpp/ParameterServerGetInt64.h>
#include <rclcpp/ParameterServerSetInt64.h>

#include "rclcpp/dds_impl/ParameterServerGetInt64Request_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetInt64Response_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetInt64Request_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetInt64Response_convert.h"

#include <rclcpp/ParameterServerGetBool.h>
#include <rclcpp/ParameterServerSetBool.h>

#include "rclcpp/dds_impl/ParameterServerGetBoolRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetBoolResponse_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetBoolRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetBoolResponse_convert.h"

#include <rclcpp/ParameterServerHas.h>

#include "rclcpp/dds_impl/ParameterServerHasRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerHasResponse_convert.h"

#include <rclcpp/ParameterServerDelete.h>

#include "rclcpp/dds_impl/ParameterServerDeleteRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerDeleteResponse_convert.h"


#include <iostream>
#include <memory>

namespace rclcpp
{
    namespace parameter
    {

    class ParameterServerHandler
    {

        private:
            std::map<std::string, boost::any> values_;

        public:
            typedef std::shared_ptr< ParameterServerHandler > Ptr;
            // Compile-time polimorphism does not work with std::bind, use
            // a different method name for every type
            bool get_param_string(rclcpp::ParameterServerGetString::Request::ConstPtr req,
                                  rclcpp::ParameterServerGetString::Response::Ptr res)
            {
                std::string s = boost::any_cast<std::string>(this->values_[req->param]);
                res->value = s;
                return true;
            }
 
            bool set_param_string(rclcpp::ParameterServerSetString::Request::ConstPtr req,
                                  rclcpp::ParameterServerSetString::Response::Ptr res)
            {
                this->values_[req->param] = boost::any_cast<std::string>(req->value);
                return true;
            }
 
            bool get_param_int64(rclcpp::ParameterServerGetInt64::Request::ConstPtr req,
                                 rclcpp::ParameterServerGetInt64::Response::Ptr res)
            {
                res->value = boost::any_cast<int64_t>(this->values_[req->param]);
                return true;
            }
 
            bool set_param_int64(rclcpp::ParameterServerSetInt64::Request::ConstPtr req,
                                 rclcpp::ParameterServerSetInt64::Response::Ptr res)
            {
                this->values_[req->param] = boost::any_cast<int64_t>(req->value);
                return true;
            }
 
            bool get_param_bool(rclcpp::ParameterServerGetBool::Request::ConstPtr req,
                                rclcpp::ParameterServerGetBool::Response::Ptr res)
            {
                res->value = boost::any_cast<uint8_t>(this->values_[req->param]);
                return true;
            }
 
            bool set_param_bool(rclcpp::ParameterServerSetBool::Request::ConstPtr req,
                                rclcpp::ParameterServerSetBool::Response::Ptr res)
            {
                this->values_[req->param] = boost::any_cast<uint8_t>(req->value);
                return true;
            }

            bool has_param(rclcpp::ParameterServerHas::Request::ConstPtr req,
                           rclcpp::ParameterServerHas::Response::Ptr res)
            {
                auto end = this->values_.end();
                res->value = this->values_.find(req->param) != end;
                return true;
            }

            bool delete_param(rclcpp::ParameterServerDelete::Request::ConstPtr req,
                              rclcpp::ParameterServerDelete::Response::Ptr res)
            {
                this->values_.erase(req->param);
                return true;
            }
    };

    class ParameterServer
    {
        public:
            typedef std::shared_ptr< ParameterServer > Ptr;
            ParameterServer(
                rclcpp::parameter::ParameterServerHandler::Ptr parameter_server_handler,
                typename rclcpp::service::Service<ParameterServerGetString>::Ptr parameter_server_get_string,
                typename rclcpp::service::Service<ParameterServerSetString>::Ptr parameter_server_set_string,
                typename rclcpp::service::Service<ParameterServerGetInt64>::Ptr parameter_server_get_int64,
                typename rclcpp::service::Service<ParameterServerSetInt64>::Ptr parameter_server_set_int64,
                typename rclcpp::service::Service<ParameterServerGetBool>::Ptr parameter_server_get_bool,
                typename rclcpp::service::Service<ParameterServerSetBool>::Ptr parameter_server_set_bool,
                typename rclcpp::service::Service<ParameterServerHas>::Ptr parameter_server_has,
                typename rclcpp::service::Service<ParameterServerDelete>::Ptr parameter_server_delete
            ) : parameter_server_handler_(parameter_server_handler),
                parameter_server_get_string_(parameter_server_get_string),
                parameter_server_set_string_(parameter_server_set_string),
                parameter_server_get_int64_(parameter_server_get_int64),
                parameter_server_set_int64_(parameter_server_set_int64),
                parameter_server_get_bool_(parameter_server_get_bool),
                parameter_server_has_(parameter_server_has),
                parameter_server_delete_(parameter_server_delete)
            {
            }

        private:
            rclcpp::parameter::ParameterServerHandler::Ptr parameter_server_handler_;
            rclcpp::service::Service<ParameterServerGetString>::Ptr parameter_server_get_string_;
            rclcpp::service::Service<ParameterServerSetString>::Ptr parameter_server_set_string_;

            rclcpp::service::Service<ParameterServerGetInt64>::Ptr parameter_server_get_int64_;
            rclcpp::service::Service<ParameterServerSetInt64>::Ptr parameter_server_set_int64_;

            rclcpp::service::Service<ParameterServerGetBool>::Ptr parameter_server_get_bool_;
            rclcpp::service::Service<ParameterServerSetBool>::Ptr parameter_server_set_bool_;

            rclcpp::service::Service<ParameterServerHas>::Ptr parameter_server_has_;
            rclcpp::service::Service<ParameterServerDelete>::Ptr parameter_server_delete_;
    };
    }
}
#endif
